#!/usr/bin/env bash

set -euo pipefail

OPENPI_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OPENPI_LOCAL_RUNTIME="${OPENPI_REPO_ROOT}/local_runtime"

export OPENPI_REPO_ROOT
export OPENPI_LOCAL_RUNTIME
export OPENPI_DATA_HOME="${OPENPI_LOCAL_RUNTIME}/cache/openpi"
export HF_LEROBOT_HOME="${OPENPI_LOCAL_RUNTIME}/datasets"
export PATH="${HOME}/.local/bin:${PATH}"

# Avoid reserving nearly all VRAM on a workstation during policy startup.
export XLA_PYTHON_CLIENT_PREALLOCATE="false"

mkdir -p \
  "${OPENPI_LOCAL_RUNTIME}/cache/openpi" \
  "${OPENPI_LOCAL_RUNTIME}/datasets" \
  "${OPENPI_LOCAL_RUNTIME}/logs" \
  "${OPENPI_LOCAL_RUNTIME}/models/current" \
  "${OPENPI_LOCAL_RUNTIME}/models/previous" \
  "${OPENPI_LOCAL_RUNTIME}/policy_records"
