#!/usr/bin/env python3
"""Builds src/geo_data.h from Natural Earth vectors.

Fetches the public-domain Natural Earth 1:10m coastline, minor
islands and admin-0 country boundary lines (GeoJSON from the
natural-earth-vector GitHub mirror), clips them to a European
bounding box, simplifies each polyline with Douglas-Peucker,
quantizes to 1/400 degree fixed point, and emits the result as int16
arrays in a C header the firmware compiles into flash.

Run from anywhere; it writes firmware/src/geo_data.h next to this
script's parent. Downloads are cached in the system temp dir so
re-runs (e.g. to retune the tolerance) are instant.

  ./make_geo_data.py [--tolerance 0.006] [--scale 400]
"""

import argparse
import json
import math
import os
import sys
import tempfile
import urllib.request

BASE = ("https://raw.githubusercontent.com/nvkelso/natural-earth-vector/"
        "master/geojson/")
# The coast set merges the mainland coastline with the minor islands
# (Natural Earth splits them; the Danish archipelago around CARL lives
# mostly in the minor-islands file).
SOURCES = {
    "coast": ["ne_10m_coastline.geojson", "ne_10m_minor_islands.geojson"],
    "border": ["ne_10m_admin_0_boundary_lines_land.geojson"],
}

# European bounding box: generously covers both homes (GORSSEL 52.2N
# 6.2E, CARL 55.7N 12.5E) out well past the 240 NM maximum range.
LON_MIN, LON_MAX = -15.0, 30.0
LAT_MIN, LAT_MAX = 35.0, 72.0


def fetch(filename, cache_dir):
    path = os.path.join(cache_dir, filename)
    if not os.path.exists(path):
        url = BASE + filename
        print(f"fetching {url}")
        with urllib.request.urlopen(url) as response:
            data = response.read()
        with open(path, "wb") as f:
            f.write(data)
    with open(path, "r") as f:
        return json.load(f)


def line_strings(geojson):
    """Yields every polyline in the collection as a list of (lon, lat).

    Polygon rings (the minor-islands file) come out as closed
    polylines — GeoJSON rings repeat their first vertex last, and
    Douglas-Peucker always keeps both endpoints, so they stay closed.
    """
    for feature in geojson["features"]:
        geom = feature["geometry"]
        if geom["type"] == "LineString":
            yield geom["coordinates"]
        elif geom["type"] == "MultiLineString":
            yield from geom["coordinates"]
        elif geom["type"] == "Polygon":
            yield from geom["coordinates"]
        elif geom["type"] == "MultiPolygon":
            for polygon in geom["coordinates"]:
                yield from polygon


def clip_runs(points):
    """Splits a polyline into runs of in-box points.

    One neighbor vertex just outside the box is kept at each end of a
    run (clamped near the box) so lines exit the visible area cleanly
    instead of stopping at the last inside vertex. Homes sit hundreds
    of NM inside the box, so the clamp distortion is never rendered.
    """
    def inside(p):
        return LON_MIN <= p[0] <= LON_MAX and LAT_MIN <= p[1] <= LAT_MAX

    def clamp(p):
        return (min(max(p[0], LON_MIN - 2.0), LON_MAX + 2.0),
                min(max(p[1], LAT_MIN - 2.0), LAT_MAX + 2.0))

    runs, run = [], []
    for i, p in enumerate(points):
        if inside(p):
            if not run and i > 0:
                run.append(clamp(points[i - 1]))  # Entry vertex.
            run.append(tuple(p))
        elif run:
            run.append(clamp(p))  # Exit vertex.
            runs.append(run)
            run = []
    if run:
        runs.append(run)
    return runs


def simplify(points, tolerance):
    """Iterative Douglas-Peucker.

    Distances are measured in degrees-of-latitude units with longitude
    scaled by cos(mean latitude), so the tolerance is isotropic in
    ground distance across the box.
    """
    if len(points) < 3:
        return list(points)
    mean_lat = sum(p[1] for p in points) / len(points)
    kx = math.cos(math.radians(mean_lat))
    pts = [(p[0] * kx, p[1]) for p in points]

    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        ax, ay = pts[a]
        bx, by = pts[b]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        worst, worst_dist = -1, tolerance
        for i in range(a + 1, b):
            px, py = pts[i]
            if norm == 0.0:
                dist = math.hypot(px - ax, py - ay)
            else:
                dist = abs(dx * (ay - py) - dy * (ax - px)) / norm
            if dist > worst_dist:
                worst, worst_dist = i, dist
        if worst >= 0:
            keep[worst] = True
            stack.append((a, worst))
            stack.append((worst, b))
    return [p for p, k in zip(points, keep) if k]


def quantize(points, scale):
    """Rounds to fixed point and drops consecutive duplicates."""
    out = []
    for lon, lat in points:
        q = (round(lat * scale), round(lon * scale))
        if not out or q != out[-1]:
            out.append(q)
    return out


def build(geojson, tolerance, scale):
    polylines = []
    for coords in line_strings(geojson):
        for run in clip_runs(coords):
            q = quantize(simplify(run, tolerance), scale)
            # Drop empty leftovers and specks whose whole extent is
            # under two quantization units — they would render as a
            # single flickering pixel at best.
            if len(q) < 2:
                continue
            lats = [p[0] for p in q]
            lons = [p[1] for p in q]
            if max(lats) - min(lats) < 2 and max(lons) - min(lons) < 2:
                continue
            polylines.append(q)
    return polylines


def emit_set(f, name, polylines):
    total = sum(len(p) for p in polylines)
    f.write(f"// {name}: {len(polylines)} polylines, {total} vertices,"
            f" {total * 4} bytes.\n")
    f.write(f"constexpr int k{name}Lines = {len(polylines)};\n")
    f.write(f"constexpr uint16_t k{name}Len[] = {{\n")
    lens = [str(len(p)) for p in polylines]
    for i in range(0, len(lens), 16):
        f.write("    " + ", ".join(lens[i:i + 16]) + ",\n")
    f.write("};\n")
    f.write(f"constexpr int16_t k{name}Pts[][2] = {{  // {{lat, lon}}\n")
    flat = [f"{{{lat},{lon}}}" for p in polylines for lat, lon in p]
    for i in range(0, len(flat), 8):
        f.write("    " + ",".join(flat[i:i + 8]) + ",\n")
    f.write("};\n\n")
    return total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tolerance", type=float, default=0.006,
                        help="Douglas-Peucker tolerance in degrees")
    parser.add_argument("--scale", type=float, default=400.0,
                        help="fixed-point units per degree")
    args = parser.parse_args()

    cache_dir = os.path.join(tempfile.gettempdir(), "natural-earth-cache")
    os.makedirs(cache_dir, exist_ok=True)

    sets = {}
    for key, filenames in SOURCES.items():
        sets[key] = []
        for filename in filenames:
            sets[key] += build(fetch(filename, cache_dir), args.tolerance,
                               args.scale)

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "src", "geo_data.h")
    with open(out_path, "w") as f:
        f.write(f"""\
// Generated by tools/make_geo_data.py — do not edit by hand.
//
// European coastlines and country borders from Natural Earth 1:10m
// (public domain): ne_10m_coastline plus ne_10m_minor_islands (the
// Coast set) and ne_10m_admin_0_boundary_lines_land (the Border set),
// clipped to lon {LON_MIN:g}..{LON_MAX:g} / lat {LAT_MIN:g}..{LAT_MAX:g}, \
Douglas-Peucker simplified
// at {args.tolerance:g} deg and quantized to 1/{args.scale:g} degree \
fixed point. Each set
// is a flat vertex array of {{lat, lon}} int16 pairs plus per-polyline
// lengths. The arrays are const, so they live in flash (memory-mapped
// on the ESP32-S3), not RAM.

#pragma once

#include <stdint.h>

namespace geodata {{

// Degrees per fixed-point unit: lat = pts[i][0] * kDegPerUnit, etc.
constexpr float kDegPerUnit = {1.0 / args.scale!r}f;

""")
        totals = {}
        totals["coast"] = emit_set(f, "Coast", sets["coast"])
        totals["border"] = emit_set(f, "Border", sets["border"])
        f.write("}  // namespace geodata\n")

    grand = sum(totals.values())
    print(f"coast:  {len(sets['coast'])} polylines, "
          f"{totals['coast']} vertices ({totals['coast'] * 4 / 1024:.1f} KB)")
    print(f"border: {len(sets['border'])} polylines, "
          f"{totals['border']} vertices ({totals['border'] * 4 / 1024:.1f} KB)")
    print(f"total:  {grand} vertices, {grand * 4 / 1024:.1f} KB -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
