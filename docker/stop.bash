#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[docker] Stopping bringup stack"

docker compose -f "${SCRIPT_DIR}/docker-compose.yml" down "$@"
