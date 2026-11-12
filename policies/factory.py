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
RESNET18_IMAGENET = "ResNet18_Weights.IMAGENET1K_V1"
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


def robot_policy_features(meta_features: dict, lang_dim: int | None = None, state_dim: int | None = None):
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.utils.feature_utils import dataset_to_policy_features

    feats = dataset_to_policy_features(meta_features)
    inputs = {k: v for k, v in feats.items() if v.type is not FeatureType.ACTION}
    outputs = {k: v for k, v in feats.items() if v.type is FeatureType.ACTION}
    if state_dim:  # derived state (e.g. joints + EE position, policies/state.py)
        inputs["observation.state"] = PolicyFeature(type=FeatureType.STATE, shape=(state_dim,))
    if lang_dim:
        inputs[LANG_KEY] = PolicyFeature(type=FeatureType.ENV, shape=(lang_dim,))
    return inputs, outputs


def make_config(kind: str, meta_features: dict, image_size: int, lang_dim: int | None = None,
                size: str = "small", chunk_size: int = 50, temporal_ensemble: float | None = 0.01,
                state_dim: int | None = None, lr: float | None = None, pretrained: bool = False,
                lr_backbone: float | None = None):
    """size="tiny" halves the transformer width (wave 2, CPU); "small" is the CPU-trainable variant used for the pipeline proof; "paper" restores the
    ACT paper/LeRobot defaults (dim 512, ffn 3200) for GPU training on real data.

    pretrained=False: ResNet-18 from random init (waves 1-2). pretrained=True: torchvision's
    ImageNet ResNet-18 (`ResNet18_Weights.IMAGENET1K_V1`). torchvision fetches it from
    download.pytorch.org; where that host is blocked, place `resnet18-f37072fd.pth` (sha256
    f37072fd...) in `$TORCH_HOME/hub/checkpoints/` and torchvision loads it from there
    (`python -m policies.pretrained --check`). Note LeRobot's ResNet uses FrozenBatchNorm2d in both
    cases, so a random-init backbone has frozen, untrained BatchNorm statistics.
    """
    from lerobot.configs.types import NormalizationMode

    if kind not in POLICY_KINDS:
        raise ValueError(f"kind must be one of {POLICY_KINDS}")
    inputs, outputs = robot_policy_features(meta_features, lang_dim if kind == "act_lang" else None, state_dim)
    if kind in ("act", "act_lang"):
        from lerobot.policies.act.configuration_act import ACTConfig

        dims = {
            "small": dict(dim_model=256, n_heads=8, dim_feedforward=1024, n_encoder_layers=4,
                          n_decoder_layers=1, n_vae_encoder_layers=4),
            # ~1.8x faster per step on CPU than small (transformer dominates ACT's step time)
            "tiny": dict(dim_model=128, n_heads=8, dim_feedforward=512, n_encoder_layers=4,
                         n_decoder_layers=1, n_vae_encoder_layers=4),
        }.get(size, {})
        cfg = ACTConfig(
            input_features=inputs, output_features=outputs, device="cpu",
            chunk_size=chunk_size,
            n_action_steps=1 if temporal_ensemble is not None else chunk_size,
            temporal_ensemble_coeff=temporal_ensemble,
            pretrained_backbone_weights=RESNET18_IMAGENET if pretrained else None,
            optimizer_lr=lr or 1e-4, optimizer_lr_backbone=lr_backbone or lr or 1e-4,
            normalization_mapping={"VISUAL": NormalizationMode.MEAN_STD, "STATE": NormalizationMode.MEAN_STD,
                                   "ACTION": NormalizationMode.MEAN_STD, "ENV": NormalizationMode.IDENTITY},
            **dims,
        )
    else:
        if lr_backbone:
            raise ValueError("lr_backbone is an ACT-only option")
        from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig

        crop = int(image_size * 0.875) // 4 * 4
        extra = {"small": dict(down_dims=(128, 256, 512)), "tiny": dict(down_dims=(64, 128, 256))}.get(size, {})
        cfg = DiffusionConfig(
            input_features=inputs, output_features=outputs, device="cpu",
            n_obs_steps=2, horizon=16, n_action_steps=8,
            crop_shape=(crop, crop), pretrained_backbone_weights=RESNET18_IMAGENET if pretrained else None,
            use_group_norm=not pretrained,  # LeRobot: GroupNorm replacement is for from-scratch only
            noise_scheduler_type="DDIM", num_train_timesteps=100, num_inference_steps=10,
            **({"optimizer_lr": lr} if lr else {}),
            **extra,
        )
    return cfg


def with_imagenet_image_stats(stats: dict, camera_keys) -> dict:
    """Pretrained backbones expect ImageNet-normalised inputs (as LeRobot's use_imagenet_stats)."""
    import torch

    out = dict(stats)
    for c in camera_keys:
        out[c] = {**stats.get(c, {}),
                  "mean": torch.tensor(IMAGENET_MEAN, dtype=torch.float32).view(3, 1, 1),
                  "std": torch.tensor(IMAGENET_STD, dtype=torch.float32).view(3, 1, 1)}
    return out


def make_policy(cfg, dataset_stats: dict, backbone: dict | None = None):
    """Policy + LeRobot pre/post processors (normalisation lives in the processors in LeRobot 0.7).
    backbone: optional from-scratch encoder spec, e.g. {"name": "small_cnn", "stride": 16}
    (policies/backbones.py); None keeps LeRobot's ResNet-18."""
    from policies.backbones import apply_backbone
    from lerobot.policies.factory import make_pre_post_processors

    if cfg.type == "act":
        from lerobot.policies.act.modeling_act import ACTPolicy as P
    else:
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy as P
    policy = apply_backbone(P(cfg), backbone)
    pre, post = make_pre_post_processors(cfg, dataset_stats=dataset_stats)
    return policy, pre, post


def config_summary(cfg) -> dict:
    d = asdict(cfg)
    return {k: v for k, v in d.items() if isinstance(v, (int, float, str, bool, type(None), list, tuple))}
