#!/usr/bin/env bash
# Start the TRON platform. Default: in-process local_sim adapter (no Isaac Sim / ROS 2 needed).
#   backend/scripts/run.sh                       # local kinematic adapter
#   TRON_ADAPTER=external backend/scripts/run.sh # wait for the ROS 2 collector + Isaac collector to POST events
set -euo pipefail
cd "$(dirname "$0")/../.."
if [ ! -x .venv/bin/uvicorn ]; then
  python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt
fi
exec .venv/bin/uvicorn backend.app.main:app --host "${TRON_HOST:-127.0.0.1}" --port "${TRON_PORT:-8000}"
