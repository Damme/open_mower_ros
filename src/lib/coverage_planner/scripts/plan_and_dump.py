#!/usr/bin/env python3
"""
plan_and_dump.py - call the coverage planner service from a GPX map and save the
result as JSON for inspection.

GPX naming scheme (tracks or routes), zone index N increasing from 0:
    mowing_area_<N>_area            -> the zone outline (required)
    mowing_area_<N>_obstacle_<M>    -> obstacle M of that zone (0..k, optional)

Workflow
--------
1. Start the planner node (advertises slic3r_coverage_planner/plan_path):

       rosrun <your_pkg> coverage_planner _visualize_plan:=false

2. Run this script (separate, sourced terminal):

       rosrun <your_pkg> plan_and_dump.py --gpx map.gpx --zone 0 --out /tmp/plan.json
       rosrun <your_pkg> plan_and_dump.py --gpx map.gpx --zone 1 --angle 30 \
              --distance 0.22 --outline-count 2

   With no --gpx it uses a built-in 10x6 m test field (one obstacle) so you can
   smoke-test the service immediately.

Coordinates: GPX lat/lon are projected to LOCAL METRES (equirectangular about an
origin). Default origin = the zone area's first vertex; pass --datum LAT LON to
use the same origin as the robot. The chosen origin is recorded in the output
JSON ("origin": [lat, lon]). Output format matches the node's debug_dump_file
({outline, holes, paths:[{is_outline, points}]}) so visualize_plan.py reads it.

NOTE on names: service TYPE is from the message package (default
"coverage_planner", from the C++ include "coverage_planner/PlanPath.h"); service
NAME is "slic3r_coverage_planner/plan_path". Override with --srv-pkg / --service.
"""

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET

import rospy
from geometry_msgs.msg import Polygon, Point32

_EARTH_R = 6371000.0  # mean Earth radius (m); good to ~cm over a garden


# ----- pure (non-ROS) helpers, unit-testable -------------------------------

def _localname(tag):
    """Strip an XML namespace: '{http://...}trk' -> 'trk'."""
    return tag.split("}", 1)[-1]


def parse_gpx(path):
    """Return {name: [(lat, lon), ...]} for every <trk> and <rte> in the GPX."""
    root = ET.parse(path).getroot()
    feats = {}
    for el in root.iter():
        if _localname(el.tag) not in ("trk", "rte"):
            continue
        name = None
        pts = []
        for child in el.iter():
            ln = _localname(child.tag)
            if ln == "name" and name is None:
                name = (child.text or "").strip()
            elif ln in ("trkpt", "rtept"):
                try:
                    pts.append((float(child.attrib["lat"]),
                                float(child.attrib["lon"])))
                except (KeyError, ValueError):
                    pass
        if name and len(pts) >= 3:
            feats.setdefault(name, pts)
    return feats


def to_local(points_ll, origin):
    """Equirectangular projection of [(lat, lon), ...] to local [[x, y], ...]."""
    lat0, lon0 = origin
    clat = math.cos(math.radians(lat0))
    return [[_EARTH_R * math.radians(lon - lon0) * clat,
             _EARTH_R * math.radians(lat - lat0)] for lat, lon in points_ll]


def field_from_gpx(path, zone, datum=None):
    """Extract (outline_xy, holes_xy, origin_latlon) for the given zone index."""
    feats = parse_gpx(path)
    area_name = "mowing_area_%d_area" % zone
    if area_name not in feats:
        avail = ", ".join(sorted(feats)) or "(none)"
        raise KeyError("no track/route named '%s' in GPX. Found: %s"
                       % (area_name, avail))
    area_ll = feats[area_name]

    obs_re = re.compile(r"^mowing_area_%d_obstacle_(\d+)$" % zone)
    obstacles = sorted(((int(m.group(1)), pts)
                        for name, pts in feats.items()
                        for m in [obs_re.match(name)] if m),
                       key=lambda t: t[0])

    origin = tuple(datum) if datum else area_ll[0]
    outline = to_local(area_ll, origin)
    holes = [to_local(pts, origin) for _, pts in obstacles]
    return outline, holes, origin


def default_field():
    """A 10 x 6 m rectangle with one 2 x 2 m square obstacle (origin unset)."""
    outline = [[0, 0], [10, 0], [10, 6], [0, 6]]
    holes = [[[4, 2], [6, 2], [6, 4], [4, 4]]]
    return outline, holes, None


# ----- ROS plumbing --------------------------------------------------------

def import_srv(pkg):
    try:
        mod = __import__(pkg + ".srv", fromlist=["PlanPath", "PlanPathRequest"])
        return mod.PlanPath, mod.PlanPathRequest
    except Exception as exc:  # noqa: BLE001
        rospy.logfatal("Could not import %s.srv (PlanPath). Built and sourced? "
                       "Override with --srv-pkg. Error: %s", pkg, exc)
        sys.exit(2)


def polygon_from(points_xy):
    poly = Polygon()
    for xy in points_xy:
        poly.points.append(Point32(x=float(xy[0]), y=float(xy[1]), z=0.0))
    return poly


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gpx", help="GPX map file; omit for the built-in test field")
    ap.add_argument("--zone", type=int, default=0,
                    help="zone index N -> mowing_area_N_area (default 0)")
    ap.add_argument("--datum", type=float, nargs=2, metavar=("LAT", "LON"),
                    help="projection origin; default = zone area's first vertex")
    ap.add_argument("--distance", type=float, default=0.22,
                    help="tool/cut width in metres (req.distance)")
    ap.add_argument("--angle", type=float, default=0.0,
                    help="mow direction in DEGREES (req.angle)")
    ap.add_argument("--outline-count", type=int, default=2,
                    help="perimeter passes (req.outline_count)")
    ap.add_argument("--outer-offset", type=float, default=0.0,
                    help="first perimeter inset; 0 -> planner uses blade radius")
    ap.add_argument("--fill", choices=["linear", "concentric"], default="linear")
    ap.add_argument("--out", default="/tmp/plan.json", help="output JSON path")
    ap.add_argument("--service", default="slic3r_coverage_planner/plan_path",
                    help="advertised service name")
    ap.add_argument("--srv-pkg", default="coverage_planner",
                    help="message package holding PlanPath.srv")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for the service")
    args = ap.parse_args()

    rospy.init_node("plan_and_dump", anonymous=True)
    PlanPath, PlanPathRequest = import_srv(args.srv_pkg)

    origin = None
    if args.gpx:
        try:
            outline, holes, origin = field_from_gpx(args.gpx, args.zone, args.datum)
        except (KeyError, ET.ParseError, OSError) as exc:
            rospy.logfatal("GPX zone %d: %s", args.zone, exc)
            sys.exit(2)
        rospy.loginfo("GPX zone %d: outline=%d pts, %d obstacle(s), origin=%s",
                      args.zone, len(outline), len(holes), origin)
    else:
        outline, holes, origin = default_field()
        rospy.loginfo("No --gpx: using built-in test field.")

    rospy.loginfo("Waiting for service '%s' ...", args.service)
    try:
        rospy.wait_for_service(args.service, timeout=args.timeout)
    except rospy.ROSException:
        rospy.logfatal("Service '%s' not available after %.0fs. Planner running?",
                       args.service, args.timeout)
        sys.exit(1)
    call = rospy.ServiceProxy(args.service, PlanPath)

    req = PlanPathRequest()
    req.distance = args.distance
    req.angle = args.angle
    req.outline_count = args.outline_count
    req.outer_offset = args.outer_offset
    req.outline = polygon_from(outline)
    req.holes = [polygon_from(h) for h in holes]
    if args.fill == "concentric":
        req.fill_type = getattr(PlanPathRequest, "FILL_CONCENTRIC", 1)
    else:
        req.fill_type = getattr(PlanPathRequest, "FILL_LINEAR", 0)

    try:
        resp = call(req)
    except rospy.ServiceException as exc:
        rospy.logfatal("Service call failed: %s", exc)
        sys.exit(1)

    out = {"outline": outline, "holes": holes,
           "origin": list(origin) if origin else None, "paths": []}
    n_outline = n_poses = 0
    for p in resp.paths:
        pts = [[ps.pose.position.x, ps.pose.position.y] for ps in p.path.poses]
        out["paths"].append({"is_outline": bool(p.is_outline), "points": pts})
        n_outline += 1 if p.is_outline else 0
        n_poses += len(pts)

    with open(args.out, "w") as f:
        json.dump(out, f)
    rospy.loginfo("Wrote %s : %d path(s) (%d outline, %d fill), %d poses.",
                  args.out, len(resp.paths), n_outline,
                  len(resp.paths) - n_outline, n_poses)


if __name__ == "__main__":
    main()