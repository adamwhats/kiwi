#!/bin/bash
set -e

PI_HOST="dev@10.42.0.2"
LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_DIR="~/kiwi_ws/src/kiwi"

echo "Stopping pi container..."
ssh $PI_HOST "docker exec kiwi-pi-1 pkill -2 -f 'ros2 launch' 2>/dev/null || true"
sleep 5
ssh $PI_HOST "cd ~/kiwi_ws/src/kiwi && docker compose stop -t 10 pi"

echo "Copying files to Pi..."
rsync -avz --delete "$LOCAL_DIR/" "$PI_HOST:$REMOTE_DIR/"

echo "Rebuilding and launching pi container..."
ssh $PI_HOST "cd ~/kiwi_ws/src/kiwi && docker compose up --build -d pi"

echo "Deploy complete"
