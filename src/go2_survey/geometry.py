"""Geodetic distance/bearing helpers and angle normalization."""

from math import asin, atan2, cos, degrees, pi, radians, sin, sqrt

EARTH_RADIUS_M = 6_371_000


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two (lat, lon) points, in meters."""
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlam = radians(lon2 - lon1)
    a = sin(dphi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(dlam / 2) ** 2
    return 2 * EARTH_RADIUS_M * asin(sqrt(a))


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from (lat1, lon1) toward (lat2, lon2), degrees [0, 360)."""
    phi1, phi2 = radians(lat1), radians(lat2)
    dlam = radians(lon2 - lon1)
    x = sin(dlam) * cos(phi2)
    y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dlam)
    return (degrees(atan2(x, y)) + 360) % 360


def normalize_angle(angle: float) -> float:
    """Normalize an angle to (-180, 180]."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def project_along_leg(
    s_lat: float,
    s_lon: float,
    e_lat: float,
    e_lon: float,
    p_lat: float,
    p_lon: float,
) -> tuple[float, float]:
    """Project point P onto the leg S->E; return (along_track_m, cross_track_m).

    Uses a local equirectangular approximation about S (meters east/north).
    For the short legs here (<100 m) the curvature error is sub-mm. `along`
    is the signed distance from S in the S->E direction (so it grows
    monotonically with forward progress and ignores cross-track wobble);
    `cross` is the absolute perpendicular offset from the leg line. Returns
    (0.0, 0.0) for a degenerate (zero-length) leg.
    """
    m_per_deg_lat = EARTH_RADIUS_M * pi / 180.0
    m_per_deg_lon = m_per_deg_lat * cos(radians(s_lat))
    ex = (e_lon - s_lon) * m_per_deg_lon
    ey = (e_lat - s_lat) * m_per_deg_lat
    px = (p_lon - s_lon) * m_per_deg_lon
    py = (p_lat - s_lat) * m_per_deg_lat
    leg_len = sqrt(ex * ex + ey * ey)
    if leg_len < 1e-9:
        return (0.0, 0.0)
    ux, uy = ex / leg_len, ey / leg_len
    along = px * ux + py * uy
    cross = abs(px * (-uy) + py * ux)
    return (along, cross)
