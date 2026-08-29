"""Policy-rate to servo-rate position interpolation."""

from __future__ import annotations

from collections.abc import Sequence


def interpolate_position(
    start: Sequence[float],
    target: Sequence[float],
    steps: int = 25,
) -> list[tuple[float, ...]]:
    """Return ``steps`` evenly spaced points, including the target.

    With the configured 5 Hz policy and 125 Hz servo loop, ``steps=25`` gives
    one command every 8 ms. The starting position is not repeated.
    """
    if len(start) != len(target):
        raise ValueError("start and target must have the same dimension")
    if steps < 1:
        raise ValueError("steps must be positive")
    start_values = tuple(float(value) for value in start)
    deltas = tuple(float(end) - begin for begin, end in zip(start_values, target, strict=True))
    return [
        tuple(begin + delta * index / steps for begin, delta in zip(start_values, deltas, strict=True))
        for index in range(1, steps + 1)
    ]
