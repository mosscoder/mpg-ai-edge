"""Logging helpers shared across go2_survey modules."""

from __future__ import annotations

import logging

_default_logger = logging.getLogger(__name__)


def log_banner(
    message: str,
    level: str = "info",
    char: str = "=",
    width: int = 60,
    logger: logging.Logger | None = None,
) -> None:
    """Log a centered banner at the given level."""
    logger = logger or _default_logger
    border = char * width
    padded = f" {message} ".center(width, char)
    log_fn = getattr(logger, level)
    log_fn(border)
    log_fn(padded)
    log_fn(border)
