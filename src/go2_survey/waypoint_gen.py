"""Generate lawnmower leg-endpoint waypoints from a point or polygon input.

Used by the ``go2-survey make-waypoints`` CLI. Reads a single GeoJSON or
KML file describing either:

  * a Point — requires ``--side-len-m``; produces a square coverage area
    centered on the point, swept by parallel legs spaced ``--leg-space-m``
    apart;
  * a Polygon — ``--side-len-m`` ignored; the polygon is **boxified**:
    reduced to its oriented bounding box (long axis along the legs),
    short axis rounded up to a whole number of ``--leg-space-m`` swaths,
    swept by ``width / spacing`` identical legs each spanning the full
    long-axis extent (15 m short axis at 5 m spacing → exactly 3 legs).
    The grid is a perfect rectangle covering 100% of the polygon; legs
    overrun the boundary where the polygon is narrower than its box;
  * multiple features — if exactly one is a polygon, the polygon is used
    and the rest are ignored (a Google Earth KML often carries a stray
    pin); all-point inputs collapse to their centroid and are treated as
    a Point (so ``--side-len-m`` must also be supplied).

Only each leg's two **endpoints** are emitted — the line-survey walking
strategy captures along the leg at a fixed distance interval at run time,
so intermediate points don't belong in the route. ``--bearing-deg``
rotates the legs clockwise from E–W — e.g. ``--bearing-deg 30`` tilts the
legs 30° clockwise. Unset, point input keeps E–W and polygon input
auto-aligns the legs to the polygon's longest edge (longest legs, fewest
corner turns — turns are the slow part of a survey).

Endpoints are emitted in serpentine order so the walk snakes leg-to-leg:
leg 1 L→R, leg 2 R→L, etc. Consecutive corners form the legs and the
short cross-step connectors between them. ``--start-corner`` picks which
compass corner of the grid wp_001 sits on (default south) — the same
serpentine walked from that corner.

Math is done in the user-specified projected CRS (``--epsg N``, expects
units in meters). Output is always EPSG:4326 (lon/lat) for compatibility
with the ``waypoints.geojson`` loader.
"""

from __future__ import annotations

import json
import logging
import math
import re
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

    # Multi-feature: a single polygon among the features wins (Google
    # Earth KMLs often carry a pin next to the area of interest); more
    # than one polygon is ambiguous; all-points collapse to a centroid
    # and run in point mode.
    polygons = [coords for kind, coords in shapes if kind == "polygon"]
    if len(polygons) == 1:
        return InputShape("polygon", polygons[0]), len(shapes)
    if len(polygons) > 1:
        raise ValueError(
            f"{len(polygons)} polygons found; provide exactly one survey area"
        )
    centers = [coords[0] for _, coords in shapes]
    cy = sum(c[0] for c in centers) / len(centers)
    cx = sum(c[1] for c in centers) / len(centers)
    return InputShape("point", [(cy, cx)]), len(shapes)


def _geojson_to_wgs84(data: dict):
    """Return a (x, y) -> (lat, lon) converter honoring a legacy `crs` member.

    RFC 7946 requires WGS-84 lon/lat and dropped `crs`, but QGIS still
    exports one when the layer lives in a projected CRS (coordinates are
    then easting/northing in that CRS). If a non-4326 EPSG is declared,
    transform on read; otherwise treat coordinates as lon/lat.
    """
    crs_name = ((data.get("crs") or {}).get("properties") or {}).get("name", "")
    m = re.search(r"EPSG:+(\d+)$", crs_name)
    if m and m.group(1) != "4326":
        to_wgs = Transformer.from_crs(
            f"EPSG:{m.group(1)}", "EPSG:4326", always_xy=True
        ).transform

        def conv(x: float, y: float) -> tuple[float, float]:
            lon, lat = to_wgs(x, y)
            return (lat, lon)

        return conv
    return lambda x, y: (y, x)


def _shapes_from_geojson(path: Path) -> list[tuple[str, list[tuple[float, float]]]]:
    data = json.loads(path.read_text())
    t = data.get("type")
    if t == "FeatureCollection":
        features = data.get("features", [])
    elif t == "Feature":
        features = [data]
    else:  # bare geometry
        features = [{"geometry": data}]

    conv = _geojson_to_wgs84(data)
    shapes: list[tuple[str, list[tuple[float, float]]]] = []
    for ft in features:
        geom = ft.get("geometry") if isinstance(ft, dict) else None
        if not geom:
            continue
        gt = geom.get("type")
        if gt == "Point":
            shapes.append(("point", [conv(*geom["coordinates"][:2])]))
        elif gt == "MultiPoint":
            for coord in geom["coordinates"]:
                shapes.append(("point", [conv(coord[0], coord[1])]))
        elif gt == "Polygon":
            ring = geom["coordinates"][0]  # outer ring
            shapes.append(("polygon", [conv(c[0], c[1]) for c in ring]))
        elif gt == "MultiPolygon":
            for poly in geom["coordinates"]:
                ring = poly[0]
                shapes.append(("polygon", [conv(c[0], c[1]) for c in ring]))
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


# ---- grid generation ------------------------------------------------------


START_CORNERS = ("south", "north", "east", "west")


def generate_grid(
    shape: InputShape,
    leg_space_m: float,
    epsg: int,
    side_len_m: float | None,
    bearing_deg: float | None = None,
    start_corner: str = "south",
) -> tuple[list[tuple[float, float]], float]:
    """Return (serpentine-ordered (lat, lon) leg endpoints, bearing used).

    Parallel legs spaced ``leg_space_m`` apart are laid out in the
    projected CRS (``EPSG:<epsg>``, units expected in meters), rotated by
    ``bearing_deg`` clockwise from the default E–W orientation, and only
    each leg's two endpoints are returned. ``bearing_deg=None`` means
    E–W for point input and longest-edge auto-alignment for polygon
    input; the resolved value is returned so callers can record it.
    ``start_corner`` picks which compass corner of the grid wp_001 sits
    on (same legs, traversal reordered). Output is back-projected to
    WGS-84.
    """
    if start_corner not in START_CORNERS:
        raise ValueError(
            f"start_corner must be one of {START_CORNERS}, got {start_corner!r}"
        )
    to_proj = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    from_proj = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)

    if shape.kind == "point":
        if side_len_m is None:
            raise ValueError("Point input requires side_len_m")
        lat, lon = shape.coords[0]
        cx, cy = to_proj.transform(lon, lat)
        used_bearing = 0.0 if bearing_deg is None else bearing_deg
        proj_pts = _grid_around_center(
            cx, cy, side_len_m, leg_space_m, used_bearing
        )
    elif shape.kind == "polygon":
        ring_proj = [to_proj.transform(lon, lat) for lat, lon in shape.coords]
        used_bearing = (
            _longest_edge_bearing(ring_proj)
            if bearing_deg is None
            else bearing_deg
        )
        proj_pts = _grid_in_polygon(ring_proj, leg_space_m, used_bearing)
    else:
        raise ValueError(f"Unknown shape kind: {shape.kind}")

    out: list[tuple[float, float]] = []
    for px, py in proj_pts:
        lon_o, lat_o = from_proj.transform(px, py)
        out.append((lat_o, lon_o))
    return _reorder_for_start(out, start_corner), used_bearing


def _reorder_for_start(
    latlons: list[tuple[float, float]], start_corner: str
) -> list[tuple[float, float]]:
    """Pick, among the equivalent serpentine traversals of the grid, the
    one whose first waypoint is extreme in the requested compass
    direction. Same legs, same spacing — only the visit order changes.

    The four traversals are: as generated, walked backwards, and the
    phase-flip of each (every leg walked end-to-start). Their first
    points are the four grid corners. Ties (e.g. exactly E–W legs make
    a whole leg southernmost) break toward the west for south/north
    starts and toward the south for east/west starts.
    """
    if len(latlons) < 2:
        return latlons

    variants = [latlons, latlons[::-1]]
    if len(latlons) % 2 == 0:
        flipped: list[tuple[float, float]] = []
        for i in range(0, len(latlons) - 1, 2):
            flipped += [latlons[i + 1], latlons[i]]
        variants += [flipped, flipped[::-1]]

    def key(pts: list[tuple[float, float]]):
        lat, lon = pts[0]
        if start_corner == "south":
            return (lat, lon)
        if start_corner == "north":
            return (-lat, lon)
        if start_corner == "west":
            return (lon, lat)
        return (-lon, lat)  # east

    return min(variants, key=key)


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


def _longest_edge_bearing(ring_proj: list[tuple[float, float]]) -> float:
    """Bearing of the polygon's longest edge in the legs-clockwise-from-E–W
    convention, normalized to [0, 180) (legs are bidirectional). Auto-
    aligning the sweep to it gives the longest legs / fewest corner turns.
    """
    best_len = 0.0
    best = 0.0
    n = len(ring_proj)
    for i in range(n):
        x1, y1 = ring_proj[i]
        x2, y2 = ring_proj[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length > best_len:
            best_len = length
            # Legs default along +x; clockwise rotation by θ maps them to
            # (cos θ, −sin θ), so θ = atan2(−dy, dx) points them along the edge.
            best = math.degrees(math.atan2(-dy, dx)) % 180.0
    # Round before re-normalizing so projection float-dust on an E–W edge
    # reads 0.0, not 179.999999….
    return round(best, 6) % 180.0


def _grid_in_polygon(
    ring_proj: list[tuple[float, float]],
    leg_space_m: float,
    bearing_deg: float,
) -> list[tuple[float, float]]:
    """Boxify the polygon and return leg endpoints in serpentine order.

    The polygon is reduced to its oriented bounding box: long axis along
    ``bearing_deg``, short axis rounded UP to the nearest whole multiple
    of ``leg_space_m`` (a 15 m short axis at 5 m spacing → exactly 3
    legs). The box is swept by ``width / leg_space_m`` identical legs,
    each inset half a spacing from the box edge and spanning the full
    long-axis extent — a perfect rectangle of equal-length legs at exact
    spacing, covering 100% of the polygon. Legs overrun the boundary
    wherever the polygon is narrower than its box (angled ends, concave
    notches); the layout plot shows both so the overrun is eyeballable.
    """
    xs = [p[0] for p in ring_proj]
    ys = [p[1] for p in ring_proj]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2

    theta = math.radians(bearing_deg)
    ct, st = math.cos(theta), math.sin(theta)

    # Polygon in the leg frame (u along-leg, v across-leg) — the inverse
    # of the clockwise leg→world rotation used on the way back out.
    ring_uv = [
        ((x - cx) * ct - (y - cy) * st, (x - cx) * st + (y - cy) * ct)
        for x, y in ring_proj
    ]
    umin = min(u for u, _ in ring_uv)
    umax = max(u for u, _ in ring_uv)
    vmin = min(v for _, v in ring_uv)
    vmax = max(v for _, v in ring_uv)

    # Box short axis = across-track extent rounded up to whole swaths
    # (the -1e-9 keeps projection float-dust from adding a phantom leg
    # when the extent is an exact multiple). Legs sit half a spacing in
    # from the box edges so the swath bands tile the box exactly.
    n = max(1, math.ceil((vmax - vmin) / leg_space_m - 1e-9))
    vc = (vmin + vmax) / 2

    pts: list[tuple[float, float]] = []
    for i in range(n):
        v = vc + (i - (n - 1) / 2) * leg_space_m
        ends = (umin, umax) if i % 2 == 0 else (umax, umin)
        for u in ends:
            # Clockwise leg→world rotation (matches _grid_around_center).
            pts.append((cx + u * ct + v * st, cy - u * st + v * ct))
    return pts


# ---- output ---------------------------------------------------------------


def write_waypoints_geojson(
    out_path: Path,
    latlons: list[tuple[float, float]],
    source_path: Path | None = None,
    epsg: int | None = None,
    bearing_deg: float | None = None,
    leg_space_m: float | None = None,
    polygon_ring: list[tuple[float, float]] | None = None,
    bearing_auto: bool = False,
    start_corner: str | None = None,
) -> None:
    """Write a FeatureCollection of Point features compatible with
    ``load_waypoints``. Generation metadata goes in the FeatureCollection
    properties block for traceability; ``polygon_ring`` ((lat, lon) outer
    ring) is stored there as ``source_polygon`` ([lon, lat] order) so
    ``plot_waypoints`` can draw the survey-area boundary.
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
        meta["bearing_deg"] = round(bearing_deg, 2)
        if bearing_auto:
            meta["bearing_source"] = "auto_longest_edge"
    if leg_space_m is not None:
        meta["leg_space_m"] = leg_space_m
    if start_corner is not None:
        meta["start_corner"] = start_corner
    if polygon_ring:
        meta["source_polygon"] = [[lon, lat] for lat, lon in polygon_ring]

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

    # Survey-area boundary, if the generator recorded one ([lon, lat] ring).
    gen = data.get("generation") or {}
    ring = gen.get("source_polygon") or []
    ring_xs = [math.radians(c[0] - lon0) * R * cos_lat0 for c in ring]
    ring_ys = [math.radians(c[1] - lat0) * R for c in ring]
    if ring_xs and (ring_xs[0], ring_ys[0]) != (ring_xs[-1], ring_ys[-1]):
        ring_xs.append(ring_xs[0])
        ring_ys.append(ring_ys[0])

    # Axis extents cover waypoints AND boundary; pad so everything sits
    # comfortably inside the frame.
    ext_minx, ext_maxx = min([minx] + ring_xs), max([maxx] + ring_xs)
    ext_miny, ext_maxy = min([miny] + ring_ys), max([maxy] + ring_ys)
    ext_span_x, ext_span_y = ext_maxx - ext_minx, ext_maxy - ext_miny
    pad = max(ext_span_x, ext_span_y) * 0.10 + 1.0

    fig, ax = plt.subplots(figsize=(8, 8 * (ext_span_y + 2 * pad) / max(ext_span_x + 2 * pad, 1e-6)))
    fig.set_size_inches(8, max(4, min(12, 8 * (ext_span_y + 2 * pad) / max(ext_span_x + 2 * pad, 1e-6))))

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

    if ring_xs:
        ax.plot(ring_xs, ring_ys, color="forestgreen", lw=1.5, alpha=0.85,
                zorder=2, label="survey area")

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

    ax.set_xlim(ext_minx - pad, ext_maxx + pad)
    ax.set_ylim(ext_miny - pad, ext_maxy + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"meters east of ({lat0:.6f}, {lon0:.6f})")
    ax.set_ylabel("meters north")
    ax.grid(True, alpha=0.3)

    mission_name = data.get("name") or geojson_path.parent.name
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
        if shape.kind == "polygon":
            print(
                f"info: {n_features} features in input; using the polygon, "
                f"ignoring the rest",
                file=sys.stderr,
            )
        else:
            print(
                f"info: {n_features} features in input; using centroid as "
                f"box center (point mode)",
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
            "info: polygon input — --side-len-m ignored, sweeping the polygon",
            file=sys.stderr,
        )

    try:
        latlons, used_bearing = generate_grid(
            shape,
            leg_space_m=args.leg_space_m,
            epsg=args.epsg,
            side_len_m=args.side_len_m,
            bearing_deg=args.bearing_deg,
            start_corner=args.start_corner,
        )
    except Exception as e:
        print(f"error: waypoint generation failed: {e}", file=sys.stderr)
        return 1

    if not latlons:
        print("warning: no waypoints generated (empty intersection)", file=sys.stderr)
        return 1

    bearing_auto = args.bearing_deg is None and shape.kind == "polygon"
    out_path = in_path.parent / "waypoints.geojson"
    write_waypoints_geojson(
        out_path,
        latlons,
        source_path=in_path,
        epsg=args.epsg,
        bearing_deg=used_bearing,
        leg_space_m=args.leg_space_m,
        polygon_ring=shape.coords if shape.kind == "polygon" else None,
        bearing_auto=bearing_auto,
        start_corner=args.start_corner,
    )
    print(
        f"wrote {len(latlons)} leg-endpoint waypoints to {out_path} "
        f"(shape={shape.kind}, epsg={args.epsg}, "
        f"leg_space={args.leg_space_m}m, bearing={used_bearing:.1f}°"
        f"{' auto: longest edge' if bearing_auto else ''}, "
        f"start={args.start_corner})"
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
