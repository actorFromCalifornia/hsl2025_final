#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="hsl2025-kobuki"

echo "[docker] Attaching to ${CONTAINER_NAME}"
docker exec -it "${CONTAINER_NAME}" /bin/bash "$@"
