#!/bin/bash
# run_test_sparkfun_imu.sh — find robot, set env, run SparkFun F9R IMU test
# Usage: bash autonomous_nav/mission/debug/run_test_sparkfun_imu.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# --- Find robot IP ---
echo "Scanning for Go2..."
ROBOT_IP=$("$REPO_ROOT/find_robot_ip.sh")

if [ -z "$ROBOT_IP" ]; then
    echo "Could not find Go2 on network. Set ROBOT_IP manually."
    export ROBOT_IP=""
else
    echo "Go2 found at $ROBOT_IP"
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

# --- Activate Python env and run ---
source ~/venvs/mpg-edge/bin/activate
python3 "$SCRIPT_DIR/test_sparkfun_imu.py"
