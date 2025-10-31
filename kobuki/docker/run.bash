#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[kobuki/docker] Starting bringup stack"

docker compose -f "${SCRIPT_DIR}/docker-compose.yml" --profile solution up 
