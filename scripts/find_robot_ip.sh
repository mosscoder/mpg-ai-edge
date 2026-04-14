#!/usr/bin/env bash
# Thin wrapper. The real discovery logic lives in go2_survey.discovery;
# this script is kept so existing callers (ops notes, muscle memory)
# keep working unchanged.
#
# Usage:
#   ./scripts/find_robot_ip.sh                   # auto-detect CIDR
#   ./scripts/find_robot_ip.sh 10.0.0.0/24       # explicit CIDR (positional, legacy)
#   ./scripts/find_robot_ip.sh --cidr 10.0.0.0/24   # or pass CLI flags directly
set -euo pipefail

case "${1:-}" in
    "")  exec go2-survey discover-ip ;;
    -*)  exec go2-survey discover-ip "$@" ;;
    *)   exec go2-survey discover-ip --cidr "$1" ;;
esac
