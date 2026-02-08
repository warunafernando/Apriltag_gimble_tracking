#!/bin/bash
# Restart backend: stop first, then start. GUI cannot send commands while backend is down.
cd "$(dirname "$0")"
pkill -f "python3 app.py" 2>/dev/null
sleep 2
nohup python3 app.py >> /tmp/gimbal_app.log 2>&1 &
echo "Backend restarted. PID: $!"
sleep 3
curl -s -o /dev/null -w "HTTP %{http_code}\n" http://127.0.0.1:5001/ && echo "Ready at http://localhost:5001"
