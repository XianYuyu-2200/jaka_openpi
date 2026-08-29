#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/runtime_env.sh"

cd "${OPENPI_REPO_ROOT}"
exec "${OPENPI_REPO_ROOT}/.venv/bin/python" examples/simple_client/main.py \
  --host 127.0.0.1 \
  --port 8000 \
  --env DROID \
  --num-steps 3 \
  --timing-file "${OPENPI_LOCAL_RUNTIME}/logs/official_smoke_timing.parquet"
