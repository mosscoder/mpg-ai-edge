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


class SportModeStateFilter(logging.Filter):
    """Suppress the 20 Hz rt/lf/sportmodestate message-log flood.

    The unitree_webrtc_connect library emits every data-channel
    message at INFO on the root logger. The sport-mode state topic
    fires at ~20 Hz and dumps IMU/position/foot payloads. Attached to
    the console + main.log handlers so the mission narrative stays
    readable; the same payloads are captured separately by the
    imu.log handler (see SportModeStateOnlyFilter).
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return "rt/lf/sportmodestate" not in record.getMessage()


class SportModeStateOnlyFilter(logging.Filter):
    """Inverse of SportModeStateFilter — keeps only rt/lf/sportmodestate.

    Attached to the per-run imu.log handler so the 20 Hz IMU/position/
    foot stream is preserved on disk for post-hoc analysis without
    polluting the main mission log.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return "rt/lf/sportmodestate" in record.getMessage()


GPS_TELEMETRY_LOGGER_NAME = "go2_survey.gps.telemetry"


# Sticky teardown flag. mission_runner.py flips this True in its finally
# block (at the *start* of teardown, before any cleanup runs). The
# WebRTCTeardownNoiseFilter consults it to suppress library-side noise
# that fires when we deliberately close the peer connection. We never
# reset to False because one CLI invocation runs exactly one mission
# and the process exits at end-of-teardown — a sticky True is the
# elegant choice here. If that contract ever changes, add a reset.
_teardown_in_progress = False


def set_teardown_in_progress(value: bool) -> None:
    """Signal the WebRTC noise filter that we're tearing down on purpose."""
    global _teardown_in_progress
    _teardown_in_progress = value


class WebRTCTeardownNoiseFilter(logging.Filter):
    """Suppress library-side stream-end noise during intentional teardown.

    Two distinct patterns fire after every successful mission:

    1. asyncio "Exception in callback ... StreamError()" + the multi-line
       traceback ending in `aiortc.mediastreams.MediaStreamError`. Source:
       unitree_webrtc_connect/webrtc_driver.py:on_track has three
       `await track.recv()` calls with no try/except. When pc.close()
       runs, recv() raises MediaStreamError (aiortc's normal end-of-stream
       signal); pyee converts it to an "error" event with no listener;
       asyncio's default handler dumps the traceback at ERROR.

    2. `aiortc.codecs.h264: H264Decoder() failed to decode, skipping
       package: Invalid data found...`. An H.264 packet was mid-decode
       when the track died; the orphaned packet looks corrupt.

    Both are cosmetic at teardown time but read as scary failures.
    Filter only applies when set_teardown_in_progress(True) has been
    called, so the same patterns mid-mission (which would indicate a
    real problem) still surface normally.
    """

    SUPPRESSED_SUBSTRINGS = (
        "MediaStreamError",
        "H264Decoder() failed to decode",
    )

    def filter(self, record: logging.LogRecord) -> bool:
        if not _teardown_in_progress:
            return True
        return not any(s in record.getMessage() for s in self.SUPPRESSED_SUBSTRINGS)


class GPSTelemetryFilter(logging.Filter):
    """Drop dense GPS telemetry records — they belong in gps.log only.

    Attached to console + main.log handlers so the mission narrative
    stays readable while the per-second JSON snapshots are routed to
    the dedicated gps.log handler.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return not record.name.startswith(GPS_TELEMETRY_LOGGER_NAME)


class GPSTelemetryOnlyFilter(logging.Filter):
    """Inverse of GPSTelemetryFilter — keeps only GPS telemetry records.

    Attached to the per-run gps.log handler so the dense JSON-per-line
    feed is preserved on disk while the rest of the mission log stays
    out of the file.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return record.name.startswith(GPS_TELEMETRY_LOGGER_NAME)


BATTERY_TELEMETRY_LOGGER_NAME = "go2_survey.battery.telemetry"


class BatteryTelemetryFilter(logging.Filter):
    """Drop the detailed per-interval battery lines — they belong in
    battery.log only. Attached to console + main.log so the narrative keeps
    just the concise `go2_survey.battery` banner (which does NOT start with
    the telemetry logger name and so passes through).
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return not record.name.startswith(BATTERY_TELEMETRY_LOGGER_NAME)


class BatteryTelemetryOnlyFilter(logging.Filter):
    """Inverse of BatteryTelemetryFilter — keeps only the detailed battery
    telemetry. Attached to the per-run battery.log handler.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return record.name.startswith(BATTERY_TELEMETRY_LOGGER_NAME)
