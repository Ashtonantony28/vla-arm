"""Frozen text encoders for the language-conditioned ACT variant (PLAN 6.3).

Backends:
  * "minilm"     sentence-transformers/all-MiniLM-L6-v2 (384-d, CPU-friendly). Needs a download
                 from huggingface.co.
  * "hashed_bow" STUB: deterministic hashed bag of words + bigrams (256-d, L2-normalised). No
                 download, no semantics: paraphrases only share whatever words they share. Used
                 when the model download is impossible (as on the build machine, where
                 huggingface.co is blocked by the egress proxy).

`get_text_encoder("auto")` tries minilm and falls back to hashed_bow with a warning; the chosen
backend name is saved into the checkpoint so eval uses the same one.
"""
from __future__ import annotations

import hashlib
import re
import warnings

import numpy as np

_TOKEN = re.compile(r"[a-z0-9]+")


class HashedBowEncoder:
    name = "hashed_bow"
    is_stub = True

    def __init__(self, dim: int = 256):
        self.dim = dim

    def _bucket(self, tok: str) -> tuple[int, float]:
        h = hashlib.blake2b(tok.encode(), digest_size=8).digest()
        v = int.from_bytes(h, "little")
        return v % self.dim, (1.0 if (v >> 63) & 1 else -1.0)

    def encode(self, texts: list[str]) -> np.ndarray:
        out = np.zeros((len(texts), self.dim), dtype=np.float32)
        for i, t in enumerate(texts):
            words = _TOKEN.findall(t.lower())
            feats = words + [f"{a}_{b}" for a, b in zip(words, words[1:])]
            # word order matters for "sort" (which colour goes to which bowl): add position-tagged
            # colour tokens so "red ... green" differs from "red ... yellow".
            colours = [w for w in words if w in {"red", "blue", "green", "yellow"}]
            feats += [f"c{k}_{c}" for k, c in enumerate(colours)]
            for f in feats:
                j, s = self._bucket(f)
                out[i, j] += s
            n = np.linalg.norm(out[i])
            if n > 0:
                out[i] /= n
        return out


class MiniLMEncoder:
    name = "minilm"
    is_stub = False
    dim = 384

    def __init__(self):
        from sentence_transformers import SentenceTransformer  # noqa: optional dependency

        self.model = SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2", device="cpu")

    def encode(self, texts: list[str]) -> np.ndarray:
        return np.asarray(self.model.encode(list(texts), normalize_embeddings=True), dtype=np.float32)


class CachedEncoder:
    """Instructions repeat across frames; cache per string (PLAN 6.4: cache the language prefix)."""

    def __init__(self, enc):
        self.enc, self.cache = enc, {}
        self.name, self.dim, self.is_stub = enc.name, enc.dim, enc.is_stub

    def encode(self, texts: list[str]) -> np.ndarray:
        missing = [t for t in dict.fromkeys(texts) if t not in self.cache]
        if missing:
            for t, e in zip(missing, self.enc.encode(missing)):
                self.cache[t] = e
        return np.stack([self.cache[t] for t in texts])


def get_text_encoder(backend: str = "auto") -> CachedEncoder:
    if backend in ("auto", "minilm"):
        try:
            return CachedEncoder(MiniLMEncoder())
        except Exception as e:  # ImportError, offline, blocked host...
            if backend == "minilm":
                raise
            warnings.warn(f"MiniLM text encoder unavailable ({type(e).__name__}: {e}); "
                          "falling back to the hashed bag-of-words STUB", stacklevel=2)
    if backend in ("auto", "hashed_bow"):
        return CachedEncoder(HashedBowEncoder())
    raise ValueError(f"unknown text encoder backend {backend}")
