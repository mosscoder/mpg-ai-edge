#!/bin/bash
# run_go2_walk.sh — set env and run Go2 walk script
# Usage: bash ~/dev/mpg-ai-edge/run_go2_walk.sh

# --- Find robot IP ---
echo "Scanning for Go2..."
ROBOT_IP=$(~/dev/mpg-ai-edge/find_robot_ip.sh)

if [ -z "$ROBOT_IP" ]; then
    echo "⚠️  Could not find Go2 on network. Set ROBOT_IP manually."
    export ROBOT_IP=""
else
    echo "✅ Go2 found at $ROBOT_IP"
    export ROBOT_IP="$ROBOT_IP"
fi

# --- Environment variables ---
export CONNECTION_MODE="LocalSTA"
export GPS_PORT="/dev/ttyACM0"
export GPS_BAUD="38400"
export EMLID_USERNAME="u65352"
export EMLID_PASSWORD="338ca"
export EMLID_MOUNTPOINT="MP15774"

echo "Environment ready. ROBOT_IP=$ROBOT_IP"

# --- Activate Python env and run walk script ---
source ~/venvs/mpg-edge/bin/activate
python3 ~/dev/mpg-ai-edge/autonomous_nav/reference/go2_walk_5m.py