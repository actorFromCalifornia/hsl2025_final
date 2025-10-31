#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export BRINGUP_MODE="bag"
echo "[docker] Starting bringup stack (mode=${BRINGUP_MODE})"

docker compose -f "${SCRIPT_DIR}/docker-compose.yml" --profile bag up --force-recreate "$@"
