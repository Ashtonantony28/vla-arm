"""Small from-scratch vision encoders for low-resolution (96x96) CPU training.

Why: with no pretrained weights available (download.pytorch.org is blocked), LeRobot's default
ResNet-18 trains from random init. At 96x96 its layer4 map is only 3x3 per camera, so ACT gets 9
image tokens per camera, each covering ~32 px -- too coarse to place a 30 mm block that is 3-15 px
wide in the image. `SmallCNN` is a 4-stage GroupNorm ResNet (~0.6-2.4 M params) with output stride
8 or 16 (12x12 or 6x6 tokens at 96 px), in the spirit of Diffusion Policy's from-scratch setting.

`apply_backbone(policy, spec)` swaps it into a constructed LeRobot ACTPolicy / DiffusionPolicy
without forking LeRobot:
* ACT: `model.backbone` (must return {"feature_map": ...}) and `model.encoder_img_feat_input_proj`;
* Diffusion: each `rgb_encoder.backbone`, with the SpatialSoftmax pool rebuilt for the new map size
  (keypoint count unchanged, so the U-Net conditioning size is unchanged).
The spec is stored in the checkpoint's `vla_arm_meta.json` ("backbone"), and `load_policy` re-applies
it before loading weights strictly.
"""
from __future__ import annotations

import torch
from torch import nn


def _gn(c: int) -> nn.GroupNorm:
    return nn.GroupNorm(max(1, c // 16), c)


class ResBlock(nn.Module):
    def __init__(self, cin: int, cout: int, stride: int):
        super().__init__()
        self.c1 = nn.Conv2d(cin, cout, 3, stride, 1, bias=False)
        self.n1 = _gn(cout)
        self.c2 = nn.Conv2d(cout, cout, 3, 1, 1, bias=False)
        self.n2 = _gn(cout)
        self.skip = (nn.Sequential(nn.Conv2d(cin, cout, 1, stride, bias=False), _gn(cout))
                     if stride != 1 or cin != cout else nn.Identity())
        self.act = nn.ReLU(inplace=True)

    def forward(self, x):
        y = self.act(self.n1(self.c1(x)))
        y = self.n2(self.c2(y))
        return self.act(y + self.skip(x))


class SmallCNN(nn.Module):
    """Stem (stride 2) + ResBlocks, each halving resolution; output stride 8 or 16."""

    def __init__(self, width: int = 32, stride: int = 16):
        super().__init__()
        if stride not in (8, 16):
            raise ValueError("stride must be 8 or 16")
        n_down = {8: 2, 16: 3}[stride]
        chans = [width * 2 ** i for i in range(n_down + 1)]
        layers = [nn.Conv2d(3, width, 3, 2, 1, bias=False), _gn(width), nn.ReLU(inplace=True)]
        for cin, cout in zip(chans[:-1], chans[1:]):
            layers.append(ResBlock(cin, cout, 2))
        layers.append(ResBlock(chans[-1], chans[-1], 1))
        self.net = nn.Sequential(*layers)
        self.out_channels = chans[-1]

    def forward(self, x):
        return self.net(x)


class _DictOut(nn.Module):
    """Wraps a module so it returns {"feature_map": y} like torchvision's IntermediateLayerGetter."""

    def __init__(self, net: nn.Module):
        super().__init__()
        self.net = net

    def forward(self, x):
        return {"feature_map": self.net(x)}


def build_backbone(spec: dict) -> SmallCNN:
    if spec.get("name") != "small_cnn":
        raise ValueError(f"unknown backbone spec {spec}")
    return SmallCNN(width=int(spec.get("width", 32)), stride=int(spec.get("stride", 16)))


def apply_backbone(policy, spec: dict | None):
    """Replace the vision backbone of a freshly constructed LeRobot policy (in place)."""
    if not spec or spec.get("name") in (None, "resnet18"):
        return policy
    cfg = policy.config
    if cfg.type == "act":
        net = build_backbone(spec)
        policy.model.backbone = _DictOut(net)
        policy.model.encoder_img_feat_input_proj = nn.Conv2d(net.out_channels, cfg.dim_model, kernel_size=1)
    elif cfg.type == "diffusion":
        from lerobot.policies.diffusion.modeling_diffusion import SpatialSoftmax
        from lerobot.policies.utils import get_output_shape

        encs = policy.diffusion.rgb_encoder
        encs = list(encs) if isinstance(encs, nn.ModuleList) else [encs]
        img_shape = next(iter(cfg.image_features.values())).shape
        hw = cfg.crop_shape or img_shape[1:]
        for enc in encs:
            enc.backbone = build_backbone(spec)
            fmap = get_output_shape(enc.backbone, (1, img_shape[0], *hw))[1:]
            enc.pool = SpatialSoftmax(fmap, num_kp=cfg.spatial_softmax_num_keypoints)
    else:
        raise ValueError(f"apply_backbone: unsupported policy type {cfg.type}")
    return policy


def count_params(m: nn.Module) -> int:
    return sum(p.numel() for p in m.parameters())


if __name__ == "__main__":  # quick size/shape report
    for s in (8, 16):
        net = SmallCNN(stride=s)
        y = net(torch.zeros(1, 3, 96, 96))
        print(f"stride {s}: out {tuple(y.shape)}, {count_params(net) / 1e6:.2f} M params")
