"""Install and verify the pretrained weights the policies use, for machines that can't download them.

    python -m policies.pretrained --resnet18 path/to/resnet18-f37072fd.pth \
        --minilm path/to/all-MiniLM-L6-v2/      # copy into the caches, verifying hashes
    python -m policies.pretrained --check        # report what is installed

* ResNet-18 (ImageNet, torchvision `ResNet18_Weights.IMAGENET1K_V1`): torchvision downloads it from
  download.pytorch.org into `$TORCH_HOME/hub/checkpoints/resnet18-f37072fd.pth` and loads it from
  there without network if present. The file name embeds the first 8 hex digits of its SHA-256;
  we check the full digest below.
* all-MiniLM-L6-v2 (sentence-transformers, 384-d): the text encoder for `act_lang`. Copied to
  `$VLA_ARM_WEIGHTS/all-MiniLM-L6-v2` (default `~/.cache/vla_arm/all-MiniLM-L6-v2`), which
  `policies.text_encoder` loads before trying the Hugging Face hub.

Wave 3 (2026-11-12): download.pytorch.org and the Hugging Face file CDN were blocked in the cloud
container, so both were fetched on another machine and copied in with this script.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
from pathlib import Path

RESNET18_FILE = "resnet18-f37072fd.pth"
RESNET18_SHA256 = "f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"
MINILM_DIR = "all-MiniLM-L6-v2"
MINILM_SHA256 = {  # Hugging Face sentence-transformers/all-MiniLM-L6-v2, main
    "model.safetensors": "53aa51172d142c89d9012cce15ae4d6cc0ca6895895114379cacb4fab128d9db",
    "tokenizer.json": "be50c3628f2bf5bb5e3a7f17b1f74611b2561a3a27eeab05e5aa30f411572037",
    "vocab.txt": "07eced375cec144d27c900241f3e339478dec958f92fddbc551f295c992038a3",
}
MINILM_FILES = ["config.json", "model.safetensors", "tokenizer.json", "tokenizer_config.json", "vocab.txt",
                "special_tokens_map.json", "modules.json", "sentence_bert_config.json",
                "config_sentence_transformers.json", "1_Pooling/config.json"]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def torch_hub_checkpoints() -> Path:
    import torch

    return Path(torch.hub.get_dir()) / "checkpoints"


def weights_root() -> Path:
    return Path(os.environ.get("VLA_ARM_WEIGHTS", Path.home() / ".cache" / "vla_arm"))


def minilm_local_dir() -> Path | None:
    d = weights_root() / MINILM_DIR
    return d if (d / "model.safetensors").exists() else None


def install_resnet18(src: Path) -> Path:
    got = sha256(src)
    if got != RESNET18_SHA256:
        raise ValueError(f"{src}: sha256 {got} != expected {RESNET18_SHA256}")
    dst = torch_hub_checkpoints() / RESNET18_FILE
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, dst)
    return dst


def install_minilm(src: Path) -> Path:
    src = Path(src)
    for name, want in MINILM_SHA256.items():
        got = sha256(src / name)
        if got != want:
            raise ValueError(f"{src / name}: sha256 {got} != expected {want}")
    dst = weights_root() / MINILM_DIR
    for name in MINILM_FILES:
        (dst / name).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src / name, dst / name)
    return dst


def check() -> dict:
    r = torch_hub_checkpoints() / RESNET18_FILE
    m = minilm_local_dir()
    return {"resnet18": str(r) if r.exists() and sha256(r) == RESNET18_SHA256 else None,
            "minilm": str(m) if m and sha256(m / "model.safetensors") == MINILM_SHA256["model.safetensors"] else None}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resnet18", type=Path)
    ap.add_argument("--minilm", type=Path)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)
    if a.resnet18:
        print("resnet18 ->", install_resnet18(a.resnet18))
    if a.minilm:
        print("minilm ->", install_minilm(a.minilm))
    if a.check or not (a.resnet18 or a.minilm):
        for k, v in check().items():
            print(f"{k}: {v or 'MISSING'}")


if __name__ == "__main__":
    main()
