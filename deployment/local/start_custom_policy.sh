#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 <training_config_name> <checkpoint_directory> [port]" >&2
  exit 2
fi

TRAINING_CONFIG="$1"
CHECKPOINT_DIRECTORY="$2"
PORT="${3:-8000}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/runtime_env.sh"

cd "${OPENPI_REPO_ROOT}"
exec "${OPENPI_REPO_ROOT}/.venv/bin/python" scripts/serve_policy.py \
  --port "${PORT}" \
  policy:checkpoint \
  --policy.config="${TRAINING_CONFIG}" \
  --policy.dir="${CHECKPOINT_DIRECTORY}"
