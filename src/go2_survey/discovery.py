"""Discover Unitree Go2 robots on the local network by WebRTC port scan.

Python wrapper around the same `nmap` + `ip route` commands that
scripts/find_robot_ip.sh has been running since the Jetson branch — not
a pure-Python reimplementation. Same binary dependency (nmap), same
Linux-only `ip` command for CIDR auto-detection, same parsing, same
exit codes. "No breaking changes" means the behavior is byte-equivalent
to the shell script it replaces.
"""

from __future__ import annotations

import logging
import subprocess
from typing import List, Optional, Set

logger = logging.getLogger(__name__)

GO2_WEBRTC_PORTS = (8081, 9991)


def detect_local_cidr() -> Optional[str]:
    """Return the IPv4 CIDR of the default-route interface, or None.

    Mirrors the shell script's detect_cidr():
        ip route show default  | awk '/default/ {print $5; exit}'
        ip -o -4 addr show dev <iface> scope global | awk '{print $4; exit}'
    """
    try:
        route = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout
    except FileNotFoundError:
        return None

    iface: Optional[str] = None
    for line in route.splitlines():
        parts = line.split()
        if parts and parts[0] == "default" and "dev" in parts:
            try:
                iface = parts[parts.index("dev") + 1]
            except IndexError:
                continue
            break
    if not iface:
        return None

    try:
        addr = subprocess.run(
            ["ip", "-o", "-4", "addr", "show", "dev", iface, "scope", "global"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout
    except FileNotFoundError:
        return None

    for line in addr.splitlines():
        parts = line.split()
        if len(parts) >= 4:
            return parts[3]
    return None


def find_robot_ips(cidr: Optional[str] = None) -> List[str]:
    """Scan a CIDR for hosts with port 8081 or 9991 open.

    Runs the exact same nmap invocation as scripts/find_robot_ip.sh:
        nmap -n -sT -p 8081,9991 --open -Pn <CIDR>

    Args:
        cidr: Network CIDR like "192.168.1.0/24". If None, auto-detected
            from `ip route show default`.

    Returns:
        Sorted list of IPv4 addresses where at least one WebRTC port was
        reported open. May be empty.

    Raises:
        RuntimeError: If `nmap` is not on PATH, or `cidr` is None and
            auto-detection fails.
    """
    if cidr is None:
        cidr = detect_local_cidr()
        if cidr is None:
            raise RuntimeError(
                "could not auto-detect local CIDR; "
                "pass --cidr explicitly (e.g. 192.168.1.0/24)"
            )

    logger.info("Scanning %s for Go2 ports 8081/9991...", cidr)
    try:
        result = subprocess.run(
            ["nmap", "-n", "-sT", "-p", "8081,9991", "--open", "-Pn", cidr],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        raise RuntimeError("nmap not found on PATH")

    return sorted(_parse_nmap_output(result.stdout))


def _parse_nmap_output(output: str) -> List[str]:
    """Extract IPs from nmap output where 8081/tcp or 9991/tcp is open.

    Equivalent to the awk block in scripts/find_robot_ip.sh:
        /Nmap scan report for/ { ip=$NF; next }
        ($1=="8081/tcp" || $1=="9991/tcp") && $2=="open" { hasPort[ip]=1 }
    """
    candidates: Set[str] = set()
    current_ip: Optional[str] = None
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Nmap scan report for "):
            current_ip = stripped.rsplit(" ", 1)[-1]
            continue
        parts = stripped.split()
        if (
            current_ip
            and len(parts) >= 2
            and parts[0] in ("8081/tcp", "9991/tcp")
            and parts[1] == "open"
        ):
            candidates.add(current_ip)
    return list(candidates)
