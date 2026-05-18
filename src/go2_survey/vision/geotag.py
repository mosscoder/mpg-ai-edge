"""Write captured frames to disk with EXIF geotag + sidecar JSON.

EXIF carries the "viewer-universal" fields (lat, lon, hMSL as
GPSAltitude, bearing as GPSImgDirection, hAcc, UTC timestamp). Sidecar
JSON next to the JPEG carries everything else: ellipsoidal height,
pDOP, fix type, numSV, correction age, headVeh, heading source,
mission context. See dev/changelog.md 2026-04-16 entry for the
EXIF-vs-sidecar split rationale.

Pillow is already a project dep; we use its EXIF helpers. The JPEG
encode path goes numpy BGR → PIL RGB → JPEG. No OpenCV dependency
needed here, even though `unitree_webrtc_connect` pulls it in.
"""

import json
import logging
import time
from dataclasses import asdict
from datetime import datetime, timezone
from fractions import Fraction
from pathlib import Path
from typing import Any

from PIL import Image
from PIL.ExifTags import Base as ExifBase
from PIL.ExifTags import GPS as ExifGPS

from go2_survey.geometry import normalize_angle
from go2_survey.gps import RTKPosition
from go2_survey.vision.frames import FrameResult

logger = logging.getLogger(__name__)


def _deg_to_dms_floats(deg: float) -> tuple:
    """Convert a signed decimal degree value to EXIF DMS (3 floats).

    EXIF stores GPSLatitude/GPSLongitude as 3 rationals (degrees,
    minutes, seconds). Passing plain floats lets PIL's EXIF writer
    pick the internal representation. The sign is carried separately
    in GPSLatitudeRef / GPSLongitudeRef ('N'/'S', 'E'/'W').
    """
    d = abs(deg)
    deg_whole = int(d)
    remainder_min = (d - deg_whole) * 60.0
    min_whole = int(remainder_min)
    seconds = (remainder_min - min_whole) * 60.0
    return (float(deg_whole), float(min_whole), seconds)


def _build_gps_ifd(
    position: RTKPosition,
    bearing: float | None,
    achieved_heading: float | None = None,
) -> dict[int, Any]:
    """Build the EXIF GPSInfo IFD dict. Keys are integer tags from
    PIL.ExifTags.GPS.

    GPSImgDirection holds the *actual* achieved heading at moment of
    capture (what the photo shows), not the navigation intent.
    `bearing` (target) goes into EXIF ImageDescription via the caller.
    Falls back to `bearing` when achieved_heading isn't available
    (e.g. nobrg captures with no rotation phase).
    """
    gps_ifd: dict[int, Any] = {}

    gps_ifd[ExifGPS.GPSLatitudeRef] = "N" if position.latitude >= 0 else "S"
    gps_ifd[ExifGPS.GPSLatitude] = _deg_to_dms_floats(position.latitude)
    gps_ifd[ExifGPS.GPSLongitudeRef] = "E" if position.longitude >= 0 else "W"
    gps_ifd[ExifGPS.GPSLongitude] = _deg_to_dms_floats(position.longitude)

    # EXIF GPSAltitude is conventionally metres above mean sea level
    # (not ellipsoidal). Use hMSL when available; fall back to
    # ellipsoidal height with a warning embedded in the sidecar only.
    altitude_msl = position.altitude_msl
    if altitude_msl is not None:
        gps_ifd[ExifGPS.GPSAltitudeRef] = 0 if altitude_msl >= 0 else 1
        gps_ifd[ExifGPS.GPSAltitude] = float(abs(altitude_msl))
    elif position.altitude is not None:
        gps_ifd[ExifGPS.GPSAltitudeRef] = 0 if position.altitude >= 0 else 1
        gps_ifd[ExifGPS.GPSAltitude] = float(abs(position.altitude))

    img_direction = achieved_heading if achieved_heading is not None else bearing
    if img_direction is not None:
        gps_ifd[ExifGPS.GPSImgDirectionRef] = "T"  # true north
        gps_ifd[ExifGPS.GPSImgDirection] = float(img_direction % 360.0)

    # Horizontal positioning error (meters) — EXIF 2.31+ GPSHPositioningError.
    gps_ifd[ExifGPS.GPSHPositioningError] = float(position.accuracy_horizontal)

    # GPS timestamp (UTC): hours/min/sec as 3 rationals + GPSDateStamp.
    dt_utc = datetime.fromtimestamp(position.timestamp, tz=timezone.utc)
    gps_ifd[ExifGPS.GPSTimeStamp] = (
        float(dt_utc.hour),
        float(dt_utc.minute),
        float(dt_utc.second + dt_utc.microsecond / 1e6),
    )
    gps_ifd[ExifGPS.GPSDateStamp] = dt_utc.strftime("%Y:%m:%d")
    gps_ifd[ExifGPS.GPSMapDatum] = "WGS-84"

    return gps_ifd


def _heading_residual(
    bearing: float | None, achieved_heading: float | None
) -> float | None:
    """Signed difference achieved − target, wrapped to (-180, 180]."""
    if bearing is None or achieved_heading is None:
        return None
    return normalize_angle(achieved_heading - bearing)


def _build_sidecar(
    frame: FrameResult,
    position: RTKPosition,
    bearing: float | None,
    achieved_heading: float | None,
    heading_source: str,
    mission_context: dict[str, Any] | None,
    extra: dict[str, Any] | None,
) -> dict[str, Any]:
    """Everything EXIF can't represent cleanly lives in the sidecar JSON.

    The `heading` block carries both target and achieved values plus
    the pre-computed residual so downstream consumers (SfM, stitching,
    geo-rectification) don't have to do their own angle-wrap math.
    Legacy `degrees_true` key is retained alongside the explicit
    `target_degrees_true` alias for back-compat with old readers.
    """
    sidecar: dict[str, Any] = {
        "schema_version": 1,
        "captured_at_unix": position.timestamp,
        "captured_at_utc": datetime.fromtimestamp(
            position.timestamp, tz=timezone.utc
        ).isoformat(),
        "frame": {
            "timestamp_unix": frame.timestamp,
            "width": frame.width,
            "height": frame.height,
        },
        "position": asdict(position),
        "heading": {
            "degrees_true": bearing,  # legacy key = target (kept for back-compat)
            "target_degrees_true": bearing,
            "achieved_degrees_true": achieved_heading,
            "residual_degrees": _heading_residual(bearing, achieved_heading),
            "source": heading_source,
        },
    }
    if mission_context:
        sidecar["mission"] = mission_context
    if extra:
        sidecar["extra"] = extra
    return sidecar


def write_geotagged_jpeg(
    frame: FrameResult,
    position: RTKPosition,
    bearing: float | None,
    out_path: Path,
    achieved_heading: float | None = None,
    heading_source: str = "unknown",
    mission_context: dict[str, Any] | None = None,
    extra: dict[str, Any] | None = None,
    jpeg_quality: int = 92,
) -> Path:
    """Write the frame as `<out_path>.jpg` with EXIF GPSInfo + sidecar JSON.

    `bearing` is the navigation target (e.g. 0.0 for b000 quadrat
    capture, None for no-bearing). `achieved_heading` is the actual
    camera direction at moment of capture; written to EXIF
    GPSImgDirection so the photo's metadata reflects what the photo
    *shows*, with the target carried in EXIF ImageDescription and
    sidecar JSON. Falls back to `bearing` for GPSImgDirection when
    `achieved_heading` is absent.

    Sidecar JSON is written to `<out_path>.json` (same basename,
    different extension). Returns the JPEG path.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # BGR -> RGB for PIL. (numpy array so no cv2 dependency.)
    bgr = frame.image
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError(
            f"Expected BGR image of shape (H, W, 3); got {bgr.shape}"
        )
    rgb = bgr[:, :, ::-1]
    img = Image.fromarray(rgb, mode="RGB")

    # Build EXIF via PIL's Exif helper.
    exif = img.getexif()
    exif[ExifBase.Make] = "Unitree"
    exif[ExifBase.Model] = "Go2"
    exif[ExifBase.Software] = "go2_survey"
    exif[ExifBase.DateTime] = datetime.fromtimestamp(
        frame.timestamp, tz=timezone.utc
    ).strftime("%Y:%m:%d %H:%M:%S")

    # ImageDescription carries the target + achieved + residual as a
    # plain ASCII string. ASCII is the only thing every EXIF reader
    # handles reliably without escaping fights; JSON-ish key=val;...
    # parses cleanly with str.split.
    if bearing is not None or achieved_heading is not None:
        parts = []
        if bearing is not None:
            parts.append(f"target_bearing_deg={bearing:.2f}")
        if achieved_heading is not None:
            parts.append(f"achieved_heading_deg={achieved_heading:.2f}")
        residual = _heading_residual(bearing, achieved_heading)
        if residual is not None:
            parts.append(f"residual_deg={residual:.2f}")
        exif[ExifBase.ImageDescription] = ";".join(parts)

    gps_ifd = _build_gps_ifd(position, bearing, achieved_heading=achieved_heading)
    exif.get_ifd(ExifBase.GPSInfo.value).update(gps_ifd)

    img.save(out_path, "JPEG", exif=exif, quality=jpeg_quality)

    sidecar = _build_sidecar(
        frame=frame,
        position=position,
        bearing=bearing,
        achieved_heading=achieved_heading,
        heading_source=heading_source,
        mission_context=mission_context,
        extra=extra,
    )
    sidecar_path = out_path.with_suffix(".json")
    sidecar_path.write_text(json.dumps(sidecar, indent=2, default=str))

    logger.info(f"Wrote geotagged capture: {out_path.name} + {sidecar_path.name}")
    return out_path


def write_frame_only_jpeg(
    frame: FrameResult,
    out_path: Path,
    mission_context: dict[str, Any] | None = None,
    extra: dict[str, Any] | None = None,
    jpeg_quality: int = 92,
) -> Path:
    """Write the frame with basic EXIF only — no GPS, no bearing.

    For lab-bench smoke tests where no RTK is available. EXIF carries
    Make/Model/Software/DateTime; sidecar JSON mirrors the geotagged
    sidecar shape but with `position: null` and `heading.source: "none"`
    so a downstream consumer that loads either kind doesn't have to
    branch on key presence.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    bgr = frame.image
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError(
            f"Expected BGR image of shape (H, W, 3); got {bgr.shape}"
        )
    rgb = bgr[:, :, ::-1]
    img = Image.fromarray(rgb, mode="RGB")

    exif = img.getexif()
    exif[ExifBase.Make] = "Unitree"
    exif[ExifBase.Model] = "Go2"
    exif[ExifBase.Software] = "go2_survey"
    exif[ExifBase.DateTime] = datetime.fromtimestamp(
        frame.timestamp, tz=timezone.utc
    ).strftime("%Y:%m:%d %H:%M:%S")

    img.save(out_path, "JPEG", exif=exif, quality=jpeg_quality)

    sidecar: dict[str, Any] = {
        "schema_version": 1,
        "captured_at_unix": frame.timestamp,
        "captured_at_utc": datetime.fromtimestamp(
            frame.timestamp, tz=timezone.utc
        ).isoformat(),
        "frame": {
            "timestamp_unix": frame.timestamp,
            "width": frame.width,
            "height": frame.height,
        },
        "position": None,
        "heading": {"degrees_true": None, "source": "none"},
    }
    if mission_context:
        sidecar["mission"] = mission_context
    if extra:
        sidecar["extra"] = extra
    sidecar_path = out_path.with_suffix(".json")
    sidecar_path.write_text(json.dumps(sidecar, indent=2, default=str))

    logger.info(f"Wrote frame-only capture: {out_path.name} + {sidecar_path.name}")
    return out_path
