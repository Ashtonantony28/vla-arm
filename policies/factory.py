"""Build LeRobot policy configs (ACT, ACT+language, Diffusion Policy) for this robot's features.

Robot features (sim and real): observation.state (6), action (6), observation.images.{wrist,scene}.
The language-conditioned ACT (PLAN 6.3) feeds a frozen text-encoder embedding of the instruction
through ACT's existing `observation.environment_state` input, which LeRobot's ACT projects to one
extra encoder token -- exactly "an extra token", with no fork of the ACT code.
"""
from __future__ import annotations

from dataclasses import asdict

LANG_KEY = "observation.environment_state"
POLICY_KINDS = ("act", "act_lang", "diffusion")


def robot_policy_features(meta_features: dict, lang_dim: int | None = None):
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.utils.feature_utils import dataset_to_policy_features

    feats = dataset_to_policy_features(meta_features)
    inputs = {k: v for k, v in feats.items() if v.type is not FeatureType.ACTION}
    outputs = {k: v for k, v in feats.items() if v.type is FeatureType.ACTION}
    if lang_dim:
        inputs[LANG_KEY] = PolicyFeature(type=FeatureType.ENV, shape=(lang_dim,))
    return inputs, outputs


def make_config(kind: str, meta_features: dict, image_size: int, lang_dim: int | None = None,
                size: str = "small", chunk_size: int = 50, temporal_ensemble: float | None = 0.01):
    """size="small" is the CPU-trainable variant used for the pipeline proof; "paper" restores the
    ACT paper/LeRobot defaults (dim 512, ffn 3200) for GPU training on real data.

    Backbones start from random init: torchvision's ImageNet weights are fetched from
    download.pytorch.org, which is blocked on the build machine. On a machine with access set
    `pretrained_backbone_weights="ResNet18_Weights.IMAGENET1K_V1"` (strongly recommended).
    """
    from lerobot.configs.types import NormalizationMode

    if kind not in POLICY_KINDS:
        raise ValueError(f"kind must be one of {POLICY_KINDS}")
    inputs, outputs = robot_policy_features(meta_features, lang_dim if kind == "act_lang" else None)
    if kind in ("act", "act_lang"):
        from lerobot.policies.act.configuration_act import ACTConfig

        dims = dict(dim_model=256, n_heads=8, dim_feedforward=1024, n_encoder_layers=4,
                    n_decoder_layers=1, n_vae_encoder_layers=4) if size == "small" else {}
        cfg = ACTConfig(
            input_features=inputs, output_features=outputs, device="cpu",
            chunk_size=chunk_size,
            n_action_steps=1 if temporal_ensemble is not None else chunk_size,
            temporal_ensemble_coeff=temporal_ensemble,
            pretrained_backbone_weights=None,
            optimizer_lr=1e-4, optimizer_lr_backbone=1e-4,
            normalization_mapping={"VISUAL": NormalizationMode.MEAN_STD, "STATE": NormalizationMode.MEAN_STD,
                                   "ACTION": NormalizationMode.MEAN_STD, "ENV": NormalizationMode.IDENTITY},
            **dims,
        )
    else:
        from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig

        crop = int(image_size * 0.875) // 4 * 4
        extra = dict(down_dims=(128, 256, 512)) if size == "small" else {}
        cfg = DiffusionConfig(
            input_features=inputs, output_features=outputs, device="cpu",
            n_obs_steps=2, horizon=16, n_action_steps=8,
            crop_shape=(crop, crop), pretrained_backbone_weights=None, use_group_norm=True,
            noise_scheduler_type="DDIM", num_train_timesteps=100, num_inference_steps=10,
            **extra,
        )
    return cfg


def make_policy(cfg, dataset_stats: dict):
    """Policy + LeRobot pre/post processors (normalisation lives in the processors in LeRobot 0.7)."""
    from lerobot.policies.factory import make_pre_post_processors

    if cfg.type == "act":
        from lerobot.policies.act.modeling_act import ACTPolicy as P
    else:
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy as P
    policy = P(cfg)
    pre, post = make_pre_post_processors(cfg, dataset_stats=dataset_stats)
    return policy, pre, post


def config_summary(cfg) -> dict:
    d = asdict(cfg)
    return {k: v for k, v in d.items() if isinstance(v, (int, float, str, bool, type(None), list, tuple))}
