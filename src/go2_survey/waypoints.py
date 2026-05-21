"""Waypoint dataclass and GeoJSON loader."""

import json
import logging
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """A navigation target point."""

    latitude: float
    longitude: float
    name: str | None = None


def load_waypoints(geojson_path: str | Path) -> list[Waypoint]:
    """Load Point/MultiPoint features from a GeoJSON FeatureCollection.

    GeoJSON stores coordinates as [longitude, latitude].
    """
    path = Path(geojson_path)
    with path.open() as f:
        data = json.load(f)

    waypoints: list[Waypoint] = []
    for i, feature in enumerate(data.get("features", [])):
        # QGIS-style exports sometimes carry features with `"geometry": null`
        # (e.g. an attribute-only row). Coalesce so .get("type") doesn't
        # crash on `None`.
        geom = feature.get("geometry") or {}
        props = feature.get("properties") or {}
        name = props.get("name") or f"waypoint_{len(waypoints) + 1}"

        geom_type = geom.get("type")
        if geom_type == "Point":
            lon, lat = geom["coordinates"][0], geom["coordinates"][1]
            waypoints.append(Waypoint(latitude=lat, longitude=lon, name=name))
        elif geom_type == "MultiPoint":
            for j, coords in enumerate(geom["coordinates"]):
                waypoints.append(
                    Waypoint(latitude=coords[1], longitude=coords[0], name=f"{name}_{j}")
                )

    logger.info(f"Loaded {len(waypoints)} waypoints from {path}")
    return waypoints
