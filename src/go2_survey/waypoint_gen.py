"""Generate lawnmower leg-endpoint waypoints from a point or polygon input.

Used by the ``go2-survey make-waypoints`` CLI. Reads a single GeoJSON or
KML file describing either:

  * a Point — requires ``--side-len-m``; produces a square coverage area
    centered on the point, swept by parallel legs spaced ``--leg-space-m``
    apart;
  * a Polygon — ``--side-len-m`` ignored; sweeps parallel legs spaced
    ``--leg-space-m`` apart across the polygon, clipped to its extent;
  * multiple features — computes their centroid and treats the result as
    a Point (so ``--side-len-m`` must also be supplied).

Only each leg's two **endpoints** are emitted — the line-survey walking
strategy captures along the leg at a fixed distance interval at run time,
so intermediate points don't belong in the route. Legs default to E–W;
``--bearing-deg`` rotates them clockwise — e.g. ``--bearing-deg 30``
tilts the legs 30° clockwise.

Endpoints are emitted in serpentine order so the walk snakes leg-to-leg:
leg 1 L→R, leg 2 R→L, etc. Consecutive corners form the legs and the
short cross-step connectors between them.

Math is done in the user-specified projected CRS (``--epsg N``, expects
units in meters). Output is always EPSG:4326 (lon/lat) for compatibility
with the ``waypoints.geojson`` loader.
"""

from __future__ import annotations

import json
import logging
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from pyproj import Transformer

logger = logging.getLogger(__name__)

KML_NS = "{http://www.opengis.net/kml/2.2}"


@dataclass
class InputShape:
    """Parsed input. Coords are (lat, lon) in WGS-84. For polygons the
    list is the outer ring (closed or open, both accepted)."""

    kind: str  # "point" or "polygon"
    coords: list[tuple[float, float]]


# ---- input parsing --------------------------------------------------------


def parse_input(path: Path) -> tuple[InputShape, int]:
    """Detect format by extension and dispatch. Returns (shape, n_features)
    where n_features is the total number of point/polygon features in the
    source file (so the caller can warn when > 1)."""
    suffix = path.suffix.lower()
    if suffix in {".geojson", ".json"}:
        shapes = _shapes_from_geojson(path)
    elif suffix == ".kml":
        shapes = _shapes_from_kml(path)
    else:
        raise ValueError(
            f"Unrecognized input extension {suffix!r}; expected "
            f".geojson, .json, or .kml"
        )

    if not shapes:
        raise ValueError(f"No Point or Polygon geometries found in {path}")

    if len(shapes) == 1:
        kind, coords = shapes[0]
        return InputShape(kind, coords), 1

    # Multi-feature: compute centroid and treat as point (per user choice).
    centers: list[tuple[float, float]] = []
    for kind, coords in shapes:
        if kind == "polygon":
            centers.append(_polygon_centroid(coords))
        else:
            centers.append(coords[0])
    cy = sum(c[0] for c in centers) / len(centers)
    cx = sum(c[1] for c in centers) / len(centers)
    return InputShape("point", [(cy, cx)]), len(shapes)


def _shapes_from_geojson(path: Path) -> list[tuple[str, list[tuple[float, float]]]]:
    data = json.loads(path.read_text())
    t = data.get("type")
    if t == "FeatureCollection":
        features = data.get("features", [])
    elif t == "Feature":
        features = [data]
    else:  # bare geometry
        features = [{"geometry": data}]

    shapes: list[tuple[str, list[tuple[float, float]]]] = []
    for ft in features:
        geom = ft.get("geometry") if isinstance(ft, dict) else None
        if not geom:
            continue
        gt = geom.get("type")
        if gt == "Point":
            lon, lat = geom["coordinates"][:2]
            shapes.append(("point", [(lat, lon)]))
        elif gt == "MultiPoint":
            for coord in geom["coordinates"]:
                shapes.append(("point", [(coord[1], coord[0])]))
        elif gt == "Polygon":
            ring = geom["coordinates"][0]  # outer ring
            shapes.append(("polygon", [(c[1], c[0]) for c in ring]))
        elif gt == "MultiPolygon":
            for poly in geom["coordinates"]:
                ring = poly[0]
                shapes.append(("polygon", [(c[1], c[0]) for c in ring]))
    return shapes


def _shapes_from_kml(path: Path) -> list[tuple[str, list[tuple[float, float]]]]:
    tree = ET.parse(path)
    root = tree.getroot()
    shapes: list[tuple[str, list[tuple[float, float]]]] = []

    # KML files in the wild are inconsistent about namespacing; try both
    # the standard kml namespace and the raw tag name.
    point_paths = [f".//{KML_NS}Point/{KML_NS}coordinates", ".//Point/coordinates"]
    poly_paths = [
        f".//{KML_NS}Polygon/{KML_NS}outerBoundaryIs/{KML_NS}LinearRing/{KML_NS}coordinates",
        ".//Polygon/outerBoundaryIs/LinearRing/coordinates",
    ]
    placemarks = list(root.iter(f"{KML_NS}Placemark")) or list(root.iter("Placemark"))

    for pm in placemarks:
        pt_el = next(
            (pm.find(p) for p in point_paths if pm.find(p) is not None), None
        )
        if pt_el is not None and pt_el.text:
            for lon, lat, *_ in _parse_kml_coords(pt_el.text):
                shapes.append(("point", [(lat, lon)]))
            continue
        poly_el = next(
            (pm.find(p) for p in poly_paths if pm.find(p) is not None), None
        )
        if poly_el is not None and poly_el.text:
            ring = _parse_kml_coords(poly_el.text)
            shapes.append(("polygon", [(c[1], c[0]) for c in ring]))
    return shapes


def _parse_kml_coords(text: str) -> list[tuple[float, ...]]:
    """KML coordinates: 'lon,lat[,alt] lon,lat[,alt] ...' (whitespace-separated)."""
    out: list[tuple[float, ...]] = []
    for token in text.strip().split():
        parts = token.split(",")
        out.append(tuple(float(x) for x in parts))
    return out


def _polygon_centroid(coords: list[tuple[float, float]]) -> tuple[float, float]:
    """Centroid of a polygon's outer ring (vertex-average, not area-weighted —
    fine for centroid-as-box-center semantics).
    """
    cy = sum(c[0] for c in coords) / len(coords)
    cx = sum(c[1] for c in coords) / len(coords)
    return (cy, cx)


# ---- grid generation ------------------------------------------------------


def generate_grid(
    shape: InputShape,
    leg_space_m: float,
    epsg: int,
    side_len_m: float | None,
    bearing_deg: float = 0.0,
) -> list[tuple[float, float]]:
    """Return a serpentine-ordered list of (lat, lon) leg endpoints in WGS-84.

    Parallel legs spaced ``leg_space_m`` apart are laid out in the
    projected CRS (``EPSG:<epsg>``, units expected in meters), rotated by
    ``bearing_deg`` clockwise from the default E–W orientation, and only
    each leg's two endpoints are returned. Output is back-projected to
    WGS-84.
    """
    to_proj = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    from_proj = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)

    if shape.kind == "point":
        if side_len_m is None:
            raise ValueError("Point input requires side_len_m")
        lat, lon = shape.coords[0]
        cx, cy = to_proj.transform(lon, lat)
        proj_pts = _grid_around_center(
            cx, cy, side_len_m, leg_space_m, bearing_deg
        )
    elif shape.kind == "polygon":
        ring_proj = [to_proj.transform(lon, lat) for lat, lon in shape.coords]
        proj_pts = _grid_in_polygon(ring_proj, leg_space_m, bearing_deg)
    else:
        raise ValueError(f"Unknown shape kind: {shape.kind}")

    out: list[tuple[float, float]] = []
    for px, py in proj_pts:
        lon_o, lat_o = from_proj.transform(px, py)
        out.append((lat_o, lon_o))
    return out


def _grid_around_center(
    cx: float,
    cy: float,
    side_len_m: float,
    leg_space_m: float,
    bearing_deg: float,
) -> list[tuple[float, float]]:
    """Parallel legs covering a square of side ``side_len_m`` centered on
    (cx, cy), spaced ``leg_space_m`` apart, rotated by ``bearing_deg``
    clockwise. Only each leg's two endpoints are returned, in serpentine
    order.

    n_legs = floor(side_len/leg_space) + 1 (covers the full extent); each
    leg spans the full side length along-track.
    """
    n = int(side_len_m / leg_space_m) + 1
    half_extent = (n - 1) * leg_space_m / 2
    offs = [-half_extent + i * leg_space_m for i in range(n)]
    # Along-track leg endpoints (full extent); collapse to one if degenerate.
    x_ends = [offs[0]] if offs[0] == offs[-1] else [offs[0], offs[-1]]

    theta = math.radians(bearing_deg)
    ct, st = math.cos(theta), math.sin(theta)

    pts: list[tuple[float, float]] = []
    for j, y in enumerate(offs):
        leg = [(x, y) for x in x_ends]
        if j % 2 == 1:
            leg.reverse()
        for x, y2 in leg:
            # Clockwise rotation: x' = x cosθ + y sinθ, y' = -x sinθ + y cosθ
            xr = x * ct + y2 * st
            yr = -x * st + y2 * ct
            pts.append((cx + xr, cy + yr))
    return pts


def _grid_in_polygon(
    ring_proj: list[tuple[float, float]],
    leg_space_m: float,
    bearing_deg: float,
) -> list[tuple[float, float]]:
    """Sweep parallel legs spaced ``leg_space_m`` apart across the polygon
    and return each leg's two endpoints (first and last in-polygon point
    along the leg), in serpentine order.

    Legs are rotated about the polygon's bbox-center so the unrotated bbox
    can be over-extended to cover all of the rotated polygon; each leg is
    then clipped to the points inside the polygon OR within
    ``leg_space_m/2`` of its boundary, and reduced to its endpoints.
    """
    xs = [p[0] for p in ring_proj]
    ys = [p[1] for p in ring_proj]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    diag_half = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) / 2

    # Over-extend so the rotated legs cover the full polygon. Add a
    # leg_space_m margin on top to catch tolerance-included edge points.
    extent = diag_half + leg_space_m
    n = int((2 * extent) / leg_space_m) + 1
    offs = [-extent + i * leg_space_m for i in range(n)]

    theta = math.radians(bearing_deg)
    ct, st = math.cos(theta), math.sin(theta)
    tol = leg_space_m / 2

    pts: list[tuple[float, float]] = []
    for j, y in enumerate(offs):
        row_pts: list[tuple[float, float]] = []
        for x in offs:
            xr = x * ct + y * st
            yr = -x * st + y * ct
            px, py = cx + xr, cy + yr
            if _point_in_polygon_with_tol(px, py, ring_proj, tol):
                row_pts.append((px, py))
        if not row_pts:
            continue
        # Keep only the leg endpoints (first & last in-polygon point).
        leg = [row_pts[0]] if len(row_pts) == 1 else [row_pts[0], row_pts[-1]]
        if j % 2 == 1:
            leg.reverse()
        pts.extend(leg)
    return pts


def _point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    """Ray casting. polygon = list of (x, y) vertices (open or closed)."""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi:
            inside = not inside
        j = i
    return inside


def _point_segment_distance(
    px: float, py: float, x1: float, y1: float, x2: float, y2: float
) -> float:
    dx, dy = x2 - x1, y2 - y1
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-30:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def _point_in_polygon_with_tol(
    x: float, y: float, polygon: list[tuple[float, float]], tol: float
) -> bool:
    if _point_in_polygon(x, y, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        j = (i + 1) % n
        if (
            _point_segment_distance(
                x, y, polygon[i][0], polygon[i][1], polygon[j][0], polygon[j][1]
            )
            <= tol
        ):
            return True
    return False


# ---- output ---------------------------------------------------------------


def write_waypoints_geojson(
    out_path: Path,
    latlons: list[tuple[float, float]],
    source_path: Path | None = None,
    epsg: int | None = None,
    bearing_deg: float | None = None,
    leg_space_m: float | None = None,
) -> None:
    """Write a FeatureCollection of Point features compatible with
    ``load_waypoints``. Generation metadata goes in the FeatureCollection
    properties block for traceability.
    """
    features: list[dict[str, Any]] = []
    for i, (lat, lon) in enumerate(latlons, start=1):
        features.append(
            {
                "type": "Feature",
                "properties": {"name": f"wp_{i:03d}"},
                "geometry": {
                    "type": "Point",
                    "coordinates": [lon, lat],
                },
            }
        )

    meta: dict[str, Any] = {}
    if source_path is not None:
        meta["source"] = str(source_path)
    if epsg is not None:
        meta["projection_epsg"] = epsg
    if bearing_deg is not None:
        meta["bearing_deg"] = bearing_deg
    if leg_space_m is not None:
        meta["leg_space_m"] = leg_space_m

    out = {
        "type": "FeatureCollection",
        "name": "generated_waypoints",
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"},
        },
        "generation": meta,
        "features": features,
    }
    out_path.write_text(json.dumps(out, indent=2))


# ---- plotting -------------------------------------------------------------


def plot_waypoints(geojson_path: Path, out_path: Path | None = None) -> Path:
    """Render a mission_layout.png from a waypoints.geojson.

    The plot uses a local-ENU projection (meters east/north from the
    centroid) so distances on the figure are physically meaningful —
    1 m east on the plot is 1 m east on the ground, regardless of
    latitude. WGS-84 lat/lon would squish horizontally at high
    latitudes and obscure the actual mission geometry.

    Shows: a dashed bounding box around all waypoints, numbered scatter
    points, straight arrows wp_N → wp_N+1 tracing the serpentine
    sequence, and a distinct marker for wp_001 so the start point is
    visually obvious.

    Returns the path to the written PNG.
    """
    # Defer the heavy import so other CLI subcommands (run, list,
    # make-waypoints) don't pay matplotlib's import cost.
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    data = json.loads(geojson_path.read_text())
    features = [
        f for f in data.get("features", [])
        if (g := f.get("geometry")) and g.get("type") == "Point"
    ]
    if not features:
        raise ValueError(f"No Point features in {geojson_path}")

    latlons: list[tuple[float, float]] = []
    names: list[str] = []
    for i, ft in enumerate(features, start=1):
        lon, lat = ft["geometry"]["coordinates"][:2]
        latlons.append((lat, lon))
        props = ft.get("properties") or {}
        names.append(props.get("name") or f"wp_{i:03d}")

    # Local ENU: project relative to centroid (small-mission approximation
    # is fine — for a 1 km square at mid-latitudes the curvature error is
    # sub-cm, well below plot resolution).
    lat0 = sum(lat for lat, _ in latlons) / len(latlons)
    lon0 = sum(lon for _, lon in latlons) / len(latlons)
    R = 6_378_137.0  # WGS-84 equatorial radius
    cos_lat0 = math.cos(math.radians(lat0))
    xs = [math.radians(lon - lon0) * R * cos_lat0 for _, lon in latlons]
    ys = [math.radians(lat - lat0) * R for lat, _ in latlons]

    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    span_x, span_y = maxx - minx, maxy - miny
    # Pad the bbox so points sit comfortably inside the frame.
    pad = max(span_x, span_y) * 0.10 + 1.0

    fig, ax = plt.subplots(figsize=(8, 8 * (span_y + 2 * pad) / max(span_x + 2 * pad, 1e-6)))
    fig.set_size_inches(8, max(4, min(12, 8 * (span_y + 2 * pad) / max(span_x + 2 * pad, 1e-6))))

    # Bounding box (dashed, tight to point extent).
    ax.add_patch(
        Rectangle(
            (minx, miny),
            span_x,
            span_y,
            fill=False,
            edgecolor="gray",
            linestyle="--",
            linewidth=1.0,
            label=f"bbox {span_x:.1f}×{span_y:.1f} m",
        )
    )

    # Arrows wp_N → wp_N+1 in sequence. Inset both ends slightly so the
    # arrowhead doesn't overlap the next waypoint's marker.
    for i in range(len(xs) - 1):
        ax.annotate(
            "",
            xy=(xs[i + 1], ys[i + 1]),
            xytext=(xs[i], ys[i]),
            arrowprops=dict(
                arrowstyle="->",
                color="steelblue",
                lw=1.0,
                alpha=0.6,
                shrinkA=6,
                shrinkB=6,
            ),
        )

    # Waypoint markers + numeric labels. Start (wp_001) gets a distinct
    # green triangle so the entry point is unambiguous.
    ax.scatter(xs[0], ys[0], marker="^", s=140, color="forestgreen",
               zorder=3, label=f"start ({names[0]})")
    ax.scatter(xs[1:-1], ys[1:-1], marker="o", s=60, color="black",
               zorder=3)
    if len(xs) > 1:
        ax.scatter(xs[-1], ys[-1], marker="s", s=80, color="firebrick",
                   zorder=3, label=f"end ({names[-1]})")

    for i, (x, y) in enumerate(zip(xs, ys), start=1):
        ax.annotate(
            str(i),
            (x, y),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
            color="black",
            zorder=4,
        )

    ax.set_xlim(minx - pad, maxx + pad)
    ax.set_ylim(miny - pad, maxy + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"meters east of ({lat0:.6f}, {lon0:.6f})")
    ax.set_ylabel("meters north")
    ax.grid(True, alpha=0.3)

    mission_name = data.get("name") or geojson_path.parent.name
    gen = data.get("generation") or {}
    subtitle_bits = [f"{len(xs)} waypoints"]
    if "leg_space_m" in gen:
        subtitle_bits.append(f"leg spacing={gen['leg_space_m']} m")
    if "bearing_deg" in gen:
        subtitle_bits.append(f"bearing={gen['bearing_deg']}°")
    if "projection_epsg" in gen:
        subtitle_bits.append(f"epsg={gen['projection_epsg']}")
    ax.set_title(f"{mission_name}\n{' | '.join(subtitle_bits)}", fontsize=10)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

    if out_path is None:
        out_path = geojson_path.parent / "mission_layout.png"
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def cmd_plot_waypoints(args) -> int:
    in_path = Path(args.input)
    if not in_path.exists():
        print(f"error: input not found: {in_path}", file=sys.stderr)
        return 1
    out = Path(args.output) if args.output else None
    try:
        written = plot_waypoints(in_path, out_path=out)
    except Exception as e:
        print(f"error: plot failed: {e}", file=sys.stderr)
        return 1
    print(f"wrote {written}")
    return 0


# ---- CLI entrypoint -------------------------------------------------------


def cmd_make_waypoints(args) -> int:
    in_path = Path(args.input)
    if not in_path.exists():
        print(f"error: input not found: {in_path}", file=sys.stderr)
        return 1

    try:
        shape, n_features = parse_input(in_path)
    except Exception as e:
        print(f"error: failed to parse {in_path}: {e}", file=sys.stderr)
        return 1

    if n_features > 1:
        print(
            f"info: {n_features} features in input; using centroid as box "
            f"center (point mode)",
            file=sys.stderr,
        )

    if shape.kind == "point" and args.side_len_m is None:
        print(
            "error: point input (single point or multi-feature centroid) "
            "requires --side-len-m",
            file=sys.stderr,
        )
        return 1
    if shape.kind == "polygon" and args.side_len_m is not None:
        print(
            "info: polygon input — --side-len-m ignored, filling polygon bbox",
            file=sys.stderr,
        )

    try:
        latlons = generate_grid(
            shape,
            leg_space_m=args.leg_space_m,
            epsg=args.epsg,
            side_len_m=args.side_len_m,
            bearing_deg=args.bearing_deg,
        )
    except Exception as e:
        print(f"error: waypoint generation failed: {e}", file=sys.stderr)
        return 1

    if not latlons:
        print("warning: no waypoints generated (empty intersection)", file=sys.stderr)
        return 1

    out_path = in_path.parent / "waypoints.geojson"
    write_waypoints_geojson(
        out_path,
        latlons,
        source_path=in_path,
        epsg=args.epsg,
        bearing_deg=args.bearing_deg,
        leg_space_m=args.leg_space_m,
    )
    print(
        f"wrote {len(latlons)} leg-endpoint waypoints to {out_path} "
        f"(shape={shape.kind}, epsg={args.epsg}, "
        f"leg_space={args.leg_space_m}m, bearing={args.bearing_deg}°)"
    )

    # Plot alongside the geojson unless the user opted out. Defer all
    # plotting errors to a warning — a missing matplotlib install or a
    # weird display backend shouldn't fail the waypoint generation.
    if not args.no_plot:
        try:
            png_path = plot_waypoints(out_path)
            print(f"wrote {png_path}")
        except Exception as e:
            print(f"warning: plot generation failed ({e}); waypoints.geojson "
                  f"still written. Retry with `go2-survey plot-waypoints "
                  f"{out_path}`.", file=sys.stderr)
    return 0
