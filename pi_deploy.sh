#!/bin/bash
set -e

PI_HOST="dev@10.42.0.2"
LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_DIR="~/xavbot_ws/src/xavbot"

echo "Stopping pi container..."
ssh $PI_HOST "cd ~/xavbot_ws/src/xavbot && docker compose stop pi"

echo "Copying files to Pi..."
rsync -avz --delete "$LOCAL_DIR/" "$PI_HOST:$REMOTE_DIR/"

echo "Rebuilding and launching pi container..."
ssh $PI_HOST "cd ~/xavbot_ws/src/xavbot && docker compose up --build -d pi"

echo "Deploy complete"
