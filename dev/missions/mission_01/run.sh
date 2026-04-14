#!/usr/bin/env bash
# Run this mission via go2-survey. Any extra args (e.g. --dry-run, -v)
# are forwarded to the CLI.
exec go2-survey run "$(cd "$(dirname "$0")" && pwd)" "$@"
