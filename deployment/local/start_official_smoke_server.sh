#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/runtime_env.sh"

cd "${OPENPI_REPO_ROOT}"
exec "${OPENPI_REPO_ROOT}/.venv/bin/python" scripts/serve_policy.py --env DROID
