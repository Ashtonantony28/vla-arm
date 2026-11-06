"""Small shared evaluation helpers."""
from __future__ import annotations

import math


def wilson_interval(successes: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    """Wilson score 95% interval for a binomial proportion (PLAN 6.6)."""
    if n == 0:
        return 0.0, 1.0
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def fmt_rate(successes: int, n: int) -> str:
    lo, hi = wilson_interval(successes, n)
    return f"{successes}/{n} = {successes / max(n, 1):.0%} (Wilson 95% CI {lo:.0%}-{hi:.0%})"
