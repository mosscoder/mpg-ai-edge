"""go2-survey CLI entry point.

Scaffold stub — real implementation lands in a later commit.
"""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    print("go2-survey CLI — not yet implemented")
    print(f"args: {argv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
