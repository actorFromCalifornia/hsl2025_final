#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[kobuki/docker] Building images via docker compose"
docker compose -f "${SCRIPT_DIR}/docker-compose.yml" build "$@"
