#!/usr/bin/env python3

from __future__ import annotations

import pathlib
import platform
import shutil
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def run(command: list[str]) -> str:
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def main() -> int:
    errors: list[str] = []
    warnings: list[str] = []

    print(f"repository: {REPO_ROOT}")
    print(f"python: {platform.python_version()}")
    if sys.version_info[:2] != (3, 11):
        errors.append("openpi environment must use Python 3.11")

    if shutil.which("nvidia-smi") is None:
        errors.append("nvidia-smi is not available")
    else:
        try:
            gpu = run(
                [
                    "nvidia-smi",
                    "--query-gpu=name,memory.total,driver_version",
                    "--format=csv,noheader",
                ]
            )
            print(f"gpu: {gpu}")
        except subprocess.CalledProcessError as exc:
            errors.append(f"nvidia-smi failed: {exc}")

    try:
        import jax
        import jax.numpy as jnp

        devices = jax.devices()
        print(f"jax: {jax.__version__}")
        print(f"jax devices: {devices}")
        if not any(device.platform == "gpu" for device in devices):
            errors.append("JAX did not detect an NVIDIA GPU")
        else:
            value = jax.jit(lambda x: jnp.sum(x * x))(jnp.arange(1024, dtype=jnp.float32))
            value.block_until_ready()
            print(f"jax CUDA smoke result: {float(value)} on {value.device}")
    except Exception as exc:
        errors.append(f"JAX CUDA smoke test failed: {exc}")

    usage = shutil.disk_usage(REPO_ROOT)
    free_gib = usage.free / (1024**3)
    print(f"free disk: {free_gib:.1f} GiB")
    if free_gib < 40:
        warnings.append("less than 40 GiB is free; model cache and datasets may fill the disk")

    try:
        commit = run(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"])
        print(f"openpi commit: {commit}")
    except subprocess.CalledProcessError as exc:
        errors.append(f"cannot read openpi git commit: {exc}")

    for warning in warnings:
        print(f"WARNING: {warning}")
    for error in errors:
        print(f"ERROR: {error}")

    if errors:
        print("environment status: FAILED")
        return 1

    print("environment status: READY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
