#!/usr/bin/env bash
set -euo pipefail

# Discover a Unitree Go2 IP by looking for hosts with either WebRTC port open.
# Usage:
#   ./find_robot_ip.sh            # auto-detect network CIDR from default route
#   ./find_robot_ip.sh 10.0.0.0/24

if ! command -v nmap >/dev/null 2>&1; then
  echo "Error: nmap is required but not installed." >&2
  exit 1
fi

detect_cidr() {
  local default_if
  local cidr

  default_if="$(ip route show default 2>/dev/null | awk '/default/ {print $5; exit}')"
  if [[ -z "${default_if}" ]]; then
    echo "Error: could not detect default network interface." >&2
    exit 1
  fi

  cidr="$(ip -o -4 addr show dev "${default_if}" scope global 2>/dev/null | awk '{print $4; exit}')"
  if [[ -z "${cidr}" ]]; then
    echo "Error: could not detect IPv4 CIDR on interface ${default_if}." >&2
    exit 1
  fi

  echo "${cidr}"
}

NETWORK_CIDR="${1:-$(detect_cidr)}"

echo "Scanning ${NETWORK_CIDR} for Go2 ports (8081 or 9991)..." >&2

SCAN_OUTPUT="$(nmap -n -sT -p 8081,9991 --open -Pn "${NETWORK_CIDR}" 2>/dev/null || true)"

CANDIDATES="$(
  printf '%s\n' "${SCAN_OUTPUT}" | awk '
    /Nmap scan report for/ { ip=$NF; next }
    ($1=="8081/tcp" || $1=="9991/tcp") && $2=="open" { hasPort[ip]=1; next }
    END {
      for (host in hasPort) print host
    }
  ' | sort -V
)"

if [[ -n "${CANDIDATES}" ]]; then
  CANDIDATE_COUNT="$(printf '%s\n' "${CANDIDATES}" | sed '/^$/d' | wc -l | tr -d ' ')"
  ROBOT_IP="$(printf '%s\n' "${CANDIDATES}" | head -n1)"

  if [[ "${CANDIDATE_COUNT}" -gt 1 ]]; then
    echo "Warning: multiple candidates found; using first: ${ROBOT_IP}" >&2
    echo "Candidates:" >&2
    printf '%s\n' "${CANDIDATES}" >&2
  fi

  echo "${ROBOT_IP}"
  exit 0
fi

echo "No host found with 8081 or 9991 open on ${NETWORK_CIDR}." >&2
echo "Tip: ensure the robot is on this network and in LocalSTA mode." >&2
exit 1
