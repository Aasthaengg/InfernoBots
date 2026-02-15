#!/usr/bin/env bash
# End-to-end: run single-fire world, log hover, then push to MindsDB.
set -e
PORT=${1:-1234}
cd "$(dirname "$0")"

# Kill stray controllers
pkill -f mavic2pro 2>/dev/null || true
pkill -f webots-controller 2>/dev/null || true

# Launch Webots; user closes when done
webots --port=$PORT worlds/mavic_2_pro_single_fire.wbt

# After Webots closes, push logs if creds are set
if [[ -n "$MINDSDB_USER" && -n "$MINDSDB_PWD" ]]; then
  cd controllers/fire_supervisor
  python push_to_mindsdb.py || true
fi
