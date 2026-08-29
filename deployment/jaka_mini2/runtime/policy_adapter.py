"""Normalize OpenPI policy responses into a native 12-D action chunk."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from .interface import ACTION_DIMENSION


def extract_action_chunk(response: Mapping[str, Any]) -> list[tuple[float, ...]]:
    """Extract and validate the server's ``actions`` array.

    The websocket policy server returns a mapping whose ``actions`` value is
    normally shaped ``[horizon, action_dim]``. A single 12-D vector is accepted
    as a one-step chunk for smoke tests.
    """
    if "actions" not in response:
        raise ValueError("policy response does not contain 'actions'")
    raw = response["actions"]
    if hasattr(raw, "tolist"):
        raw = raw.tolist()
    if not isinstance(raw, Sequence) or isinstance(raw, str | bytes):
        raise ValueError("policy actions must be a sequence")
    if raw and not isinstance(raw[0], Sequence):
        raw = [raw]
    chunk: list[tuple[float, ...]] = []
    for index, action in enumerate(raw):
        if len(action) != ACTION_DIMENSION:
            raise ValueError(f"policy action {index} must have {ACTION_DIMENSION} values, got {len(action)}")
        chunk.append(tuple(float(value) for value in action))
    if not chunk:
        raise ValueError("policy action chunk must not be empty")
    return chunk
