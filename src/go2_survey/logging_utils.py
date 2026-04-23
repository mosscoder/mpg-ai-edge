"""Logging helpers shared across go2_survey modules."""

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


class WebRTCFallbackNoiseFilter(logging.Filter):
    """Suppress the unitree_webrtc_connect legacy-SDP-probe error pair.

    The library tries an HTTP POST on port 8081 ('old method') before
    falling back to the canonical WebRTC offer path that actually
    works on our Go2 firmware. The probe failure is logged at ERROR
    severity on the root logger:

        [ERROR] root: An error occurred: HTTPConnectionPool(...
            port=8081): Max retries exceeded with url: /offer ...
        [ERROR] root: An error occurred with the old method:
            Failed to receive SDP Answer: No response

    Neither is a real failure — the library recovers on the next
    request — so we drop both. Attach to handlers so records reach
    the filter regardless of which logger emitted them.
    """

    SUPPRESSED_SUBSTRINGS = (
        "Max retries exceeded with url: /offer",
        "An error occurred with the old method:",
    )

    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return not any(s in msg for s in self.SUPPRESSED_SUBSTRINGS)
