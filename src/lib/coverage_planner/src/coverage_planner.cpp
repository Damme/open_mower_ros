//
// coverage_planner.cpp - OpenMower coverage planner (drop-in for slic3r_coverage_planner).
//
// Per zone: [1] build field  [2+3] boundary loops + obstacle rings from shared
// per-level contours (a loop wraps around an obstacle that sits in its band)
// [4] fill (serpentine cells or concentric rings)  [5] nearest-neighbour order
// [6] emit nav_msgs/Path + markers + JSON dump.
//
// Clearance model: outline/obstacle inputs are base_link traces the mower drove,
// so offset 0 retraces them; outer_offset adds an overgrowth margin. Loop runs
// are clipped at the (slightly deflated) keep-out; fill connectors are validated
// centerline-in-drivable only, no body-footprint check.
//
#include "ros/ros.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"

#include "geom.hpp"

using namespace geom;

// ---- ROS params (loaded in main) -------------------------------------------
static bool g_visualize = true;
static int g_lane_skip = 2;                // grass-recovery lane grouping (k)
static bool g_optimize_angle = false;      // true: replace req.angle with the region's principal axis
static double g_blade_off_x = 0.0, g_blade_off_y = 0.0;
static double g_tool_width = -1.0;         // cut width [m]; <= 0 = follow req.distance
static double g_overlap = 0.20;            // swath overlap, fraction of cut width [0..0.9]
static double g_clearance = -1.0;          // boundary inset [m]; < 0 = follow req.outer_offset
static double g_obstacle_clearance = -1.0; // obstacle inset [m]; < 0 = follow clearance
static int g_outline_count = -1;           // perimeter passes; < 0 = follow req.outline_count
static std::string g_fill_mode = "request";// request | serpentine | concentric
static std::string g_dump_file;
static double g_body_width = 0.39, g_body_length = 0.58;
static double g_min_turn_radius = 0.0, g_axle_from_rear = 0.11;
static double g_loop_transition = 6.0;     // ring step-over length ~ this * ring gap
static std::string g_edge_side = "right";  // boundary/obstacle edge is kept on this side of the mower
static ros::Publisher g_markers;

static double g_path_spacing = 0.1;            // emitted pose spacing + loop resample [m]
static const double kTurnCheckSpacing = 0.05;  // connector validation sample spacing [m]
static const double kClipGuard = 0.05;         // keep-out deflation before clipping paths [m]
static const double kBlendSlack = 0.01;        // blend validation tolerance outside drivable [m]

// ---- Stage 0: config ---------------------------------------------------------
// Every field is assigned in buildConfig(); the initializers below are safety
// zeros, NOT settings. Configure via ROS params / the request.
struct PlanConfig {
  double blade_diameter = 0.22;
  double swath_step = 0.22;           // blade * (1 - overlap)
  int n_perimeter_passes = 2;
  double clearance = 0.0;             // inset from the recorded edge; 0 = retrace it
  double obstacle_clearance = 0.0;    // same, from recorded obstacle loops
  double angle_deg = 0.0;
  bool concentric = false;            // req.fill_type FILL_CONCENTRIC
  std::string edge_side = "right";
  // turning geometry -> turn_footprint(), the max-gap cutoff for U-turns
  double body_width = 0.39, body_length = 0.58, min_turn_radius = 0.0, axle_from_rear = 0.11;
  double spin_radius() const { return std::hypot(body_length - axle_from_rear, body_width / 2.0); }
  double turn_footprint() const {    // width needed to reverse direction
    return min_turn_radius > 0.0 ? 2.0 * min_turn_radius + body_width : 2.0 * spin_radius();
  }
  double fill_border() const {       // inset where the fill starts (half a swath inside loop n)
    return clearance + (n_perimeter_passes > 0 ? (n_perimeter_passes - 0.5) * swath_step : 0.0);
  }
};

static PlanConfig buildConfig(const coverage_planner::PlanPathRequest &req) {
  PlanConfig c;
  // Params with sentinel defaults follow the request (see main).
  c.blade_diameter = std::max(0.01, g_tool_width > 0.0 ? g_tool_width
                                    : req.distance > 1e-6 ? req.distance
                                                          : 0.22);   // neither set: sane default
  c.swath_step = c.blade_diameter * (1.0 - g_overlap);
  c.n_perimeter_passes = g_outline_count >= 0 ? g_outline_count
                                              : std::max(0, (int)req.outline_count);
  c.body_width = g_body_width; c.body_length = g_body_length;
  c.min_turn_radius = g_min_turn_radius; c.axle_from_rear = g_axle_from_rear;
  c.clearance = g_clearance >= 0.0 ? g_clearance
              : req.outer_offset > 1e-6 ? req.outer_offset : 0.0;
  c.obstacle_clearance = g_obstacle_clearance >= 0.0 ? g_obstacle_clearance : c.clearance;
  c.angle_deg = req.angle * 180.0 / M_PI;                // mower_logic sends RADIANS, offsets folded in
  c.edge_side = (g_edge_side == "left") ? "left" : "right";
  if (g_fill_mode == "concentric")      c.concentric = true;
  else if (g_fill_mode == "serpentine") c.concentric = false;
  else c.concentric = (req.fill_type == coverage_planner::PlanPathRequest::FILL_CONCENTRIC);
  return c;
}

// ---- planner data --------------------------------------------------------------
struct Pass {
  enum Kind { PERIMETER, OBSTACLE_RING, FILL } kind;
  std::vector<Pt> pts;
  bool is_outline = false;
};

struct Swath { int row; std::vector<Pt> pts; };   // fill lane + field-global scan-row index

struct Field {
  BMultiPolygon outline;                 // pristine boundary
  BMultiPolygon mowable;                 // outline - obstacles (may be multi-lobe)
  BMultiPolygon keepout;                 // obstacles (+) obstacle_clearance, hard no-go
  BMultiPolygon keepout_clip;            // keepout deflated by kClipGuard (see shrinkKeepout)
  BMultiPolygon obstacles_raw;           // union of in-zone recorded obstacle traces
  std::vector<BPolygon> ring_obstacles;  // obstacles that qualify for rings
};

// Every component of `a` covered by `b`. Per-polygon: multipolygon-in-
// multipolygon covered_by is not reliable in Boost.
static bool allCoveredBy(const BMultiPolygon &a, const BMultiPolygon &b) {
  if (bg::is_empty(a)) return false;
  for (const auto &p : a) {
    bool c = false; try { c = bg::covered_by(p, b); } catch (...) {}
    if (!c) return false;
  }
  return true;
}

// Deflate each keep-out component by eps so paths that legitimately ride the
// keep-out boundary survive the clip (numeric jitter otherwise shreds them).
// A component too small to shrink stays raw: worst case its rings shred and
// drop as slivers -- never a path across the obstacle.
static BMultiPolygon shrinkKeepout(const BMultiPolygon &keepout, double eps) {
  BMultiPolygon out;
  for (const auto &p : keepout) {
    BMultiPolygon s = bufferPolygon(p, -eps);
    if (bg::is_empty(s)) out.push_back(p);
    else for (auto &q : s) out.push_back(std::move(q));
  }
  return out;
}

// DIAGNOSTIC ONLY -- no path surgery. A hairpin-class turn (heading change
// > 100 deg) whose swept body fan reaches into a recorded obstacle: the mower
// may bump there and firmware collision handling takes over. The recorded
// outline is trusted as drivable (it was driven), so only obstacles count.
static bool turnHitsObstacle(const Pt &a, const Pt &b, const Pt &c,
                             const BMultiPolygon &obstacles, double r) {
  double h1 = std::atan2(b.y - a.y, b.x - a.x);
  double h2 = std::atan2(c.y - b.y, c.x - b.x);
  double d = std::remainder(h2 - h1, 2.0 * M_PI);
  if (std::fabs(d) < 100.0 * M_PI / 180.0) return false;   // normal driving
  const double beta = 0.42;                                // body corner angular half-extent [rad]
  double from = h1 - (d > 0 ? beta : -beta);
  double span = d + (d > 0 ? 2.0 * beta : -2.0 * beta);
  int steps = std::max(3, (int)std::ceil(std::fabs(span) / 0.26));   // ~15 deg
  for (int i = 0; i <= steps; ++i) {
    double th = from + span * i / steps;
    double cs = std::cos(th), sn = std::sin(th);
    if (multiContains(obstacles, BPoint(b.x + r * cs, b.y + r * sn))) return true;
    if (multiContains(obstacles, BPoint(b.x + 0.5 * r * cs, b.y + 0.5 * r * sn))) return true;
  }
  return false;
}

// Scan the final passes and warn (once, with examples) where hard turns sweep
// into an obstacle. Headings over a ~0.2 m window so a corner split across
// resample vertices still registers.
static void warnObstacleTurns(const std::vector<const Pass *> &order, const Field &f,
                              const PlanConfig &cfg) {
  if (bg::is_empty(f.obstacles_raw)) return;
  const double r = cfg.spin_radius();
  int hits = 0;
  std::ostringstream where; where.precision(2); where << std::fixed;
  for (const Pass *p : order) {
    std::vector<Pt> pts = resample(p->pts, g_path_spacing);
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      const Pt &pa = pts[i >= 2 ? i - 2 : i - 1];
      const Pt &pc = pts[i + 2 < pts.size() ? i + 2 : i + 1];
      if (turnHitsObstacle(pa, pts[i], pc, f.obstacles_raw, r)) {
        if (hits < 5) where << (hits ? ", " : "") << "(" << pts[i].x << "," << pts[i].y << ")";
        ++hits;
        i += 4;                                          // one report per corner
      }
    }
  }
  if (hits)
    ROS_WARN_STREAM("coverage_planner: " << hits << " hard turn(s) sweep into an obstacle"
                    << " (mower may bump; collision handling recovers): " << where.str()
                    << (hits > 5 ? ", ..." : "") << ".");
}

// Funnel for every stacked run: clip at the deflated keep-out, drop slivers
// shorter than one cut.
static void appendClipped(std::vector<Pass> &out, Pass::Kind kind, bool is_outline,
                          const std::vector<Pt> &run, const Field &f, const PlanConfig &cfg) {
  for (auto &arc : clipPathOutside(run, f.keepout_clip)) {
    if (arc.size() < 2 || pathLength(arc) < cfg.blade_diameter) continue;   // drop slivers
    Pass p; p.kind = kind; p.is_outline = is_outline; p.pts = std::move(arc);
    out.push_back(std::move(p));
  }
}

// Connector/blend validation region: boundary clearance minus keep-outs.
// Crossing mown ground is fine. Empty on failure (fail safe: runs fragment).
static BMultiPolygon drivableRegion(const Field &f, const PlanConfig &cfg) {
  BMultiPolygon bound = bufferPolygon(f.outline, -cfg.clearance);
  if (bg::is_empty(f.keepout)) return bound;
  BMultiPolygon d; try { bg::difference(bound, f.keepout, d); } catch (...) { d.clear(); }
  return d;
}

// ---- Stage 1: build field, classify obstacles -----------------------------------
static Field buildField(const coverage_planner::PlanPathRequest &req, const PlanConfig &cfg) {
  Field f;
  std::vector<Pt> ring;
  for (const auto &p : req.outline.points) ring.emplace_back(p.x, p.y);
  f.outline = sanitize(makePolygon(ring));
  f.mowable = f.outline;
  if (bg::is_empty(f.outline)) return f;

  for (const auto &hole : req.holes) {
    std::vector<Pt> h;
    for (const auto &p : hole.points) h.emplace_back(p.x, p.y);
    if (h.size() < 3) continue;
    BMultiPolygon hp = sanitize(makePolygon(h));
    if (bg::is_empty(hp)) continue;

    bool touches = false; try { touches = bg::intersects(hp, f.outline); } catch (...) {}
    if (!touches) continue;                                  // outside the zone
    { BMultiPolygon u; try { bg::union_(f.obstacles_raw, hp, u); f.obstacles_raw = u; } catch (...) {} }

    // Cut from mowable, pre-grown by any excess of obstacle over boundary
    // clearance so the fill border lands at obstacle_clearance from the obstacle.
    double extra = std::max(0.0, cfg.obstacle_clearance - cfg.clearance);
    BMultiPolygon cut = extra > 1e-6 ? bufferPolygon(hp, extra) : hp;
    BMultiPolygon diff; try { bg::difference(f.mowable, cut, diff); } catch (...) {}
    if (!bg::is_empty(diff)) f.mowable = diff;               // ignore a cut that erases everything

    BMultiPolygon grown = bufferPolygon(hp, cfg.obstacle_clearance);   // hard keep-out
    if (!bg::is_empty(grown)) {
      if (bg::is_empty(f.keepout)) f.keepout = grown;
      else { BMultiPolygon u; bg::union_(f.keepout, grown, u); f.keepout = u; }
    }

    // Ring-eligible only if fully inside AND its clearance band clears the
    // boundary -- otherwise the outermost ring would hit the perimeter.
    bool fully_inside = true;
    for (const auto &poly : hp) {
      bool cov = false; try { cov = bg::covered_by(poly, f.outline); } catch (...) {}
      if (!cov) { fully_inside = false; break; }
    }
    if (fully_inside && allCoveredBy(grown, f.outline))
      for (const auto &poly : hp) f.ring_obstacles.push_back(poly);
  }
  f.keepout_clip = shrinkKeepout(f.keepout, kClipGuard);
  return f;
}

// ---- Stages 2+3: boundary loops + obstacle rings, merged per level ------------
// Level k contours are the boundary of R_k = (outline inset b_k) - (ring
// obstacles grown o_k). Far apart that is the classic loop k plus separate ring
// k; when an obstacle sits within a level's band the hole merges into the outer
// contour and level k becomes ONE continuous loop wrapping around the obstacle.
// A ring therefore never crosses its own level's loop line. Level 1 is exempt:
// the closest passes ride the recorded traces untouched (conflicts clip).
// Outer contours emit as PERIMETER, driven deepest-first ending at the
// boundary; hole contours emit as OBSTACLE rings, driven outermost-first ending
// at the obstacle. Winding follows edge_side through wraps automatically.
static std::vector<Pass> loopPasses(const Field &f, const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (bg::is_empty(f.outline) || cfg.n_perimeter_passes <= 0) return out;
  const int n = cfg.n_perimeter_passes;
  const bool perim_ccw = (cfg.edge_side == "right");   // CCW -> boundary on the right
  BMultiPolygon lim = bufferPolygon(drivableRegion(f, cfg), kBlendSlack);
  auto blend_ok = [&](const std::vector<Pt> &b) { return pathInside(b, lim, kTurnCheckSpacing); };

  BMultiPolygon obs;                                   // union of ring obstacles
  for (const auto &p : f.ring_obstacles) {
    BMultiPolygon u; try { bg::union_(obs, toMulti(p), u); obs = u; } catch (...) {}
  }

  std::vector<BMultiPolygon> R(n + 1);                 // R[k], k = 1..n
  for (int k = 1; k <= n; ++k) {
    BMultiPolygon base = bufferPolygon(f.outline, -(cfg.clearance + (k - 1) * cfg.swath_step));
    if (k > 1 && !bg::is_empty(obs) && !bg::is_empty(base)) {
      BMultiPolygon g = bufferPolygon(obs, cfg.obstacle_clearance + (k - 1) * cfg.swath_step);
      BMultiPolygon d; try { bg::difference(base, g, d); } catch (...) { d.clear(); }  // fail: level vanishes
      base = d;
    }
    R[k] = base;
  }

  struct Lobe { BPolygon last; std::vector<std::vector<Pt>> rings; };
  auto claim = [](std::vector<Lobe> &lobes, std::vector<char> &claimed, const BPolygon &comp) {
    for (size_t a = 0; a < lobes.size(); ++a) {
      if (claimed[a]) continue;
      bool ov = false; try { ov = bg::intersects(comp, lobes[a].last); } catch (...) {}
      if (ov) { claimed[a] = 1; return (int)a; }
    }
    return -1;
  };

  // Perimeter family: outer contours, levels 1..n (R is monotone shrinking, so
  // an empty level ends it). Runs step inner -> out.
  {
    std::vector<Lobe> lobes;
    auto finish = [&](const Lobe &L) {
      if (L.rings.empty()) return;
      std::vector<std::vector<Pt>> ordered(L.rings.rbegin(), L.rings.rend());  // deepest first
      auto sp = stackLoops(ordered, g_path_spacing, g_loop_transition, blend_ok);
      if (sp.size() < 2) return;
      appendClipped(out, Pass::PERIMETER, true, sp, f, cfg);
    };
    for (int k = 1; k <= n; ++k) {
      std::vector<BPolygon> comps;
      for (const auto &p : R[k]) if (!bg::is_empty(p)) comps.push_back(p);
      if (comps.empty()) break;                        // deeper levels are emptier
      std::vector<Lobe> next;
      std::vector<char> claimed(lobes.size(), 0);
      for (const auto &comp : comps) {
        int li = claim(lobes, claimed, comp);
        Lobe nl; nl.last = comp;
        std::vector<Pt> r = orientRing(outerRing(comp), perim_ccw);
        if (li >= 0) { nl.rings = lobes[li].rings; nl.rings.push_back(r); }
        else nl.rings = {r};                           // split -> fresh lobe
        next.push_back(std::move(nl));
      }
      for (size_t a = 0; a < lobes.size(); ++a) if (!claimed[a]) finish(lobes[a]);  // lobe ended
      lobes.swap(next);
    }
    for (const auto &L : lobes) finish(L);
  }

  // Obstacle family: hole contours of R_k for k = n..2 (absent while wrapped
  // into the outer contour, appearing as the level drops), plus the untouched
  // level-1 ring on the obstacle traces. Runs step outer -> in.
  if (!bg::is_empty(obs)) {
    std::vector<Lobe> lobes;
    auto finish = [&](const Lobe &L) {
      if (L.rings.empty()) return;
      auto sp = stackLoops(L.rings, g_path_spacing, g_loop_transition, blend_ok);  // outer -> in
      if (sp.size() < 2) return;
      appendClipped(out, Pass::OBSTACLE_RING, true, sp, f, cfg);
    };
    for (int k = n; k >= 1; --k) {
      std::vector<BPolygon> comps;                     // hole regions as polygons
      if (k == 1) {
        BMultiPolygon g1 = bufferPolygon(obs, cfg.obstacle_clearance);
        for (const auto &p : g1) if (!bg::is_empty(p)) comps.push_back(p);
      } else {
        for (const auto &p : R[k])
          for (auto &hole : innerRings(p)) {
            BPolygon hp = makePolygon(hole);           // corrects winding
            if (!bg::is_empty(hp)) comps.push_back(hp);
          }
      }
      std::vector<Lobe> next;
      std::vector<char> claimed(lobes.size(), 0);
      for (const auto &comp : comps) {
        int li = claim(lobes, claimed, comp);
        Lobe nl; nl.last = comp;
        std::vector<Pt> r = orientRing(outerRing(comp), !perim_ccw);   // obstacle on the edge side
        if (li >= 0) { nl.rings = lobes[li].rings; nl.rings.push_back(r); }
        else nl.rings = {r};                           // appeared / peanut split -> fresh lobe
        next.push_back(std::move(nl));
      }
      for (size_t a = 0; a < lobes.size(); ++a) if (!claimed[a]) finish(lobes[a]);
      lobes.swap(next);
    }
    for (const auto &L : lobes) finish(L);
  }
  return out;
}

// ---- Stage 4: fill -------------------------------------------------------------------

// Parallel swaths over `region` at `angle_deg`, spaced `step`, grouped into
// boustrophedon cells, each swath tagged with a field-global scan-row index.
// A segment continues a cell only on a 1:1 overlap with the previous row;
// split/merge/new opens a fresh cell, and an EMPTY row breaks continuity so
// cells never chain across a gap. ride_bounds (used when there are no perimeter
// loops) pins the first/last row ON the region boundary with even spacing <= step.
static std::vector<std::vector<Swath>> decomposeCells(const BMultiPolygon &region,
                                                      double angle_deg, double step,
                                                      bool ride_bounds) {
  std::vector<std::vector<Swath>> cells;
  if (bg::is_empty(region) || step <= 1e-6) return cells;

  Pt cen = centroidOf(region);
  BMultiPolygon rot = rotateMulti(region, -angle_deg, cen);
  double minx, miny, maxx, maxy; boundsOf(rot, minx, miny, maxx, maxy);
  double rad = angle_deg * M_PI / 180.0, cs = std::cos(rad), sn = std::sin(rad);
  auto back = [&](const Pt &p) {                                  // rotated frame -> world
    double dx = p.x - cen.x, dy = p.y - cen.y;
    return Pt(cen.x + dx * cs - dy * sn, cen.y + dx * sn + dy * cs);
  };

  const long cap = 100000;                                        // bad-scale guard
  std::vector<double> ys;
  double span = maxy - miny;
  if (ride_bounds && span > 1e-6) {
    long nr = std::min(cap, (long)std::max(1, (int)std::ceil(span / step - 1e-9)));
    double st = span / nr;                                        // even, <= step
    for (long i = 0; i <= nr; ++i) ys.push_back(miny + i * st);   // rows ON both bounds
  } else {
    for (double y = miny + step / 2.0; y <= maxy + 1e-9 && (long)ys.size() < cap; y += step)
      ys.push_back(y);                                            // centred rows (loops cover the rim)
  }

  struct Seg { double lo, hi; std::vector<Pt> pts; };             // lo/hi = x-extent (rotated)
  std::vector<std::vector<Seg>> stripes;
  for (double y : ys) {
    std::vector<Seg> row;
    for (auto &seg : scanLineClip(rot, y, minx, maxx)) {
      if (seg.size() < 2) continue;
      Pt a = seg.front(), b = seg.back();
      if (a.x > b.x) std::swap(a, b);
      row.push_back({a.x, b.x, {back(a), back(b)}});
    }
    std::sort(row.begin(), row.end(), [](const Seg &A, const Seg &B) { return A.lo < B.lo; });
    stripes.push_back(std::move(row));                            // empty rows too: they break cells
  }

  auto overlap = [&](const Seg &A, const Seg &B) {
    double lo = std::max(A.lo, B.lo), hi = std::min(A.hi, B.hi);
    return (hi - lo) > -0.3 * step;                               // slack bridges tiny gaps
  };
  std::vector<Seg> prev; std::vector<int> prevCell;
  for (size_t ri = 0; ri < stripes.size(); ++ri) {
    std::vector<Seg> &cur = stripes[ri];
    std::vector<int> curCell(cur.size(), -1);
    for (size_t ci = 0; ci < cur.size(); ++ci) {
      int match = -1, nmatch = 0;                            // prev segments this one overlaps
      for (size_t pi = 0; pi < prev.size(); ++pi)
        if (overlap(cur[ci], prev[pi])) { match = (int)pi; ++nmatch; }
      bool cont = false;
      if (nmatch == 1) {                                     // 1:1 both ways -> continue the cell
        int fan = 0; for (auto &c2 : cur) if (overlap(c2, prev[match])) ++fan;
        if (fan == 1) { curCell[ci] = prevCell[match]; cont = true; }
      }
      if (!cont) { curCell[ci] = (int)cells.size(); cells.push_back({}); }  // split/merge/new
      cells[curCell[ci]].push_back({(int)ri, cur[ci].pts});
    }
    prev.swap(cur); prevCell.swap(curCell);
  }
  return cells;
}

// k groups by global row (group o = rows o mod k): adjacent lanes are mowed a
// group apart so cut grass springs back. Tiny cell (<= k lanes) stays one group.
static std::vector<std::vector<Swath>> laneSkipGroups(const std::vector<Swath> &sw, int k) {
  std::vector<std::vector<Swath>> groups;
  if (k <= 1 || (int)sw.size() <= k) { groups.push_back(sw); return groups; }
  for (int o = 0; o < k; ++o) {
    std::vector<Swath> g;
    for (const auto &s : sw) if (s.row % k == o) g.push_back(s);
    if (!g.empty()) groups.push_back(std::move(g));
  }
  return groups;
}

// Canonical lane direction: fixed unit vector toward +x (tie-break +y), the
// same for every group and cell. Flip the sign here to run stripes the other way.
static Pt laneAxis(const std::vector<Pt> &lane) {
  if (lane.size() < 2) return Pt(1.0, 0.0);
  Pt d(lane.back().x - lane.front().x, lane.back().y - lane.front().y);
  double l = std::hypot(d.x, d.y); if (l < 1e-9) return Pt(1.0, 0.0);
  d.x /= l; d.y /= l;
  if (d.x < -1e-9 || (std::fabs(d.x) < 1e-9 && d.y < 0.0)) { d.x = -d.x; d.y = -d.y; }
  return d;
}

// Chain a cell's swaths into serpentine runs, one per lane_skip group. Drive
// direction = (row/k) parity -> k-wide stripes, phase-aligned field-wide.
// U-turns are fitted to the headland; one that fails even at the tightest
// radius breaks the run (nav repositions).
static std::vector<Pass> stitchSwaths(const std::vector<Swath> &sw_in,
                                      const BMultiPolygon &drivable, const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (sw_in.empty()) return out;
  const int k = std::max(1, g_lane_skip);
  double base_r = cfg.min_turn_radius > 1e-6 ? cfg.min_turn_radius : cfg.swath_step;
  double max_gap = cfg.turn_footprint() * 2.0 + k * cfg.swath_step;
  Pt pref = laneAxis(sw_in[0].pts);                          // shared by all groups/cells
  for (auto &sw : laneSkipGroups(sw_in, k)) {
    for (auto &s : sw) {                                     // orient by the row's k-band
      if (s.pts.size() < 2) continue;
      Pt d(s.pts.back().x - s.pts.front().x, s.pts.back().y - s.pts.front().y);
      double want = ((s.row / k) % 2 == 0) ? 1.0 : -1.0;
      if ((d.x * pref.x + d.y * pref.y) * want < 0.0) std::reverse(s.pts.begin(), s.pts.end());
    }
    Pass run; run.kind = Pass::FILL; run.pts = sw[0].pts;
    for (size_t i = 1; i < sw.size(); ++i) {
      std::vector<Pt> &cur = sw[i].pts;
      const Pt &a = run.pts.back();
      const Pt &ah = run.pts.size() >= 2 ? run.pts[run.pts.size() - 2] : cur.front();
      const Pt &b = cur.front();
      const Pt &bh = cur.size() >= 2 ? cur[1] : cur.back();
      // Widest U-turn whose centerline fits in `drivable`: start gentle, step
      // tighter to the mower's limit.
      double r_top   = std::max(base_r, 0.6 * dist(a, b));
      double r_floor = cfg.min_turn_radius > 1e-6 ? cfg.min_turn_radius : base_r * 0.3;
      std::vector<Pt> conn;
      bool fits = false;
      for (double rr = r_top; ; rr = std::max(r_floor, rr * 0.6)) {
        conn = makeTurn(a, ah, b, bh, rr);
        if (pathInside(conn, drivable, kTurnCheckSpacing)) { fits = true; break; }
        if (rr <= r_floor * 1.0001) break;                   // tightest tried, give up
      }
      if (dist(a, b) < max_gap && fits) {                    // gap gate rejects far-apart lanes
        for (size_t j = 1; j + 1 < conn.size(); ++j) run.pts.push_back(conn[j]);
        for (const auto &p : cur) run.pts.push_back(p);
      } else {
        out.push_back(run); run = Pass(); run.kind = Pass::FILL; run.pts = cur;  // break the run
      }
    }
    out.push_back(run);
  }
  return out;
}

// Concentric fill: nested rings instead of swaths. N = ceil(D / swath_step)
// rings over the max inward depth D, so spacing D/N is even and <= swath_step
// (no gaps); a final half-step ring closes the centre core. lane_skip groups
// rings by depth like the serpentine groups lanes. Winding follows edge_side
// like an obstacle: the uncut core stays on the configured side. Rings trace
// each lobe's OUTER boundary only -- holes are left to the obstacle rings and
// the keep-out clip, so serpentine handles interior obstacles better.
static std::vector<Pass> concentricFill(const Field &f, const BMultiPolygon &inner,
                                        const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (bg::is_empty(inner)) return out;
  double D = inscribedWidth(inner) * 0.5;              // largest inscribed radius
  if (D < 1e-6) return out;
  int N = std::max(1, (int)std::ceil(D / cfg.swath_step - 1e-9));
  double step = D / N;

  std::vector<double> depths;                          // 0, step, ..., then half-step centre core
  for (int i = 0; i < N; ++i) depths.push_back(i * step);
  depths.push_back((N - 0.5) * step);

  // Per-lobe ring stacks tracked as `inner` shrinks and splits (as in
  // loopPasses); depth index keeps lane_skip grouping aligned after splits.
  struct DRing { int depth; std::vector<Pt> ring; };
  struct Lobe { BPolygon last; std::vector<DRing> rings; };
  std::vector<Lobe> lobes, done;
  for (size_t di = 0; di < depths.size(); ++di) {
    BMultiPolygon shrunk = (depths[di] < 1e-9) ? inner : bufferPolygon(inner, -depths[di]);
    std::vector<BPolygon> comps;
    for (const auto &p : shrunk) if (!bg::is_empty(p)) comps.push_back(p);
    std::vector<Lobe> next;
    std::vector<char> claimed(lobes.size(), 0);
    for (const auto &comp : comps) {
      int li = -1;                                     // continue the lobe this ring nests in
      for (size_t a = 0; a < lobes.size(); ++a) {
        if (claimed[a]) continue;
        bool ov = false; try { ov = bg::intersects(comp, lobes[a].last); } catch (...) {}
        if (ov) { li = (int)a; claimed[a] = 1; break; }
      }
      Lobe nl; nl.last = comp;
      if (li >= 0) { nl.rings = lobes[li].rings; nl.rings.push_back({(int)di, outerRing(comp)}); }
      else nl.rings = {{(int)di, outerRing(comp)}};    // split -> fresh lobe
      next.push_back(std::move(nl));
    }
    for (size_t a = 0; a < lobes.size(); ++a) if (!claimed[a]) done.push_back(lobes[a]);
    lobes.swap(next);
  }
  for (auto &L : lobes) done.push_back(L);

  const int k = std::max(1, g_lane_skip);
  const bool want_ccw = (cfg.edge_side == "left");     // CW -> uncut core on the right
  BMultiPolygon lim = bufferPolygon(drivableRegion(f, cfg), kBlendSlack);
  auto blend_ok = [&](const std::vector<Pt> &b) { return pathInside(b, lim, kTurnCheckSpacing); };
  for (auto &L : done) {
    for (int o = 0; o < k; ++o) {                      // one atomic run per depth group
      std::vector<std::vector<Pt>> group;
      for (const auto &r : L.rings)
        if (r.depth % k == o) group.push_back(orientRing(r.ring, want_ccw));
      if (group.empty()) continue;
      auto sp = stackLoops(group, g_path_spacing, g_loop_transition, blend_ok);
      if (sp.size() < 2) continue;
      appendClipped(out, Pass::FILL, false, sp, f, cfg);
    }
  }
  return out;
}

static std::vector<Pass> fillArea(const Field &f, const BMultiPolygon &inner, const PlanConfig &cfg) {
  std::vector<Pass> out;
  if (bg::is_empty(inner)) return out;
  if (cfg.concentric) return concentricFill(f, inner, cfg);

  BMultiPolygon drivable = drivableRegion(f, cfg);
  double ang = g_optimize_angle ? principalAxisDeg(outerRing(largestComponent(inner)))
                                : cfg.angle_deg;
  double min_run = cfg.blade_diameter;                       // drop slivers shorter than one cut
  bool ride = (cfg.n_perimeter_passes == 0);                 // no loops -> rows ride the boundary
  for (auto &cell : decomposeCells(inner, ang, cfg.swath_step, ride)) {
    if (cell.empty()) continue;
    for (auto &p : stitchSwaths(cell, drivable, cfg)) {
      if (p.pts.size() >= 2 && pathLength(p.pts) >= min_run) out.push_back(std::move(p));
    }
  }
  return out;
}

// ---- Stage 5: order ---------------------------------------------------------------

// Greedy nearest-neighbour, entering each run from its built start only: fill
// runs carry the stripe direction and loops are wound edge-side; never reverse.
static void orderByNearest(std::vector<Pass> &passes, Pt start) {
  std::vector<Pass> ordered; ordered.reserve(passes.size());
  std::vector<char> used(passes.size(), 0);
  Pt cur = start;
  for (size_t n = 0; n < passes.size(); ++n) {
    int best = -1; double bestd = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < passes.size(); ++i) {
      if (used[i] || passes[i].pts.empty()) continue;
      double df = dist(cur, passes[i].pts.front());
      if (df < bestd) { bestd = df; best = (int)i; }
    }
    if (best < 0) break;
    used[best] = 1; Pass p = passes[best];
    cur = p.pts.back(); ordered.push_back(std::move(p));
  }
  passes.swap(ordered);
}

// ---- Stage 6: emit ------------------------------------------------------------------
static geometry_msgs::Quaternion yawToQuat(double yaw) {
  tf2::Quaternion q; q.setRPY(0, 0, yaw); return tf2::toMsg(q);
}

// Resample, blade->base_link shift, per-pose yaw from the segment ahead.
static coverage_planner::Path emitPath(const std_msgs::Header &h, const std::vector<Pt> &raw,
                                       bool is_outline) {
  coverage_planner::Path path; path.is_outline = is_outline; path.path.header = h;
  std::vector<Pt> pts = resample(raw, g_path_spacing);
  if (pts.size() < 2) return path;
  pts = bladePathToBase(pts, g_blade_off_x, g_blade_off_y);   // no-op if deck centred
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double yaw = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
    geometry_msgs::PoseStamped ps; ps.header = h; ps.pose.orientation = yawToQuat(yaw);
    ps.pose.position.x = pts[i].x; ps.pose.position.y = pts[i].y;
    path.path.poses.push_back(ps);
  }
  geometry_msgs::PoseStamped last; last.header = h;           // final pose keeps last heading
  last.pose.orientation = path.path.poses.empty() ? yawToQuat(0) : path.path.poses.back().pose.orientation;
  last.pose.position.x = pts.back().x; last.pose.position.y = pts.back().y;
  path.path.poses.push_back(last);
  return path;
}

// JSON dump for the offline visualizer (no-op when debug_dump_file is unset).
static void dumpPlan(const std::string &file, const coverage_planner::PlanPathRequest &req,
                     const coverage_planner::PlanPathResponse &res) {
  if (file.empty()) return;
  std::ofstream f(file.c_str()); if (!f) return; f.precision(9);
  f << "{\n  \"outline\": [";
  for (size_t i = 0; i < req.outline.points.size(); ++i)
    f << (i ? "," : "") << "[" << req.outline.points[i].x << "," << req.outline.points[i].y << "]";
  f << "],\n  \"holes\": [";
  for (size_t hh = 0; hh < req.holes.size(); ++hh) {
    f << (hh ? "," : "") << "[";
    for (size_t i = 0; i < req.holes[hh].points.size(); ++i)
      f << (i ? "," : "") << "[" << req.holes[hh].points[i].x << "," << req.holes[hh].points[i].y << "]";
    f << "]";
  }
  f << "],\n  \"paths\": [";
  for (size_t pi = 0; pi < res.paths.size(); ++pi) {
    f << (pi ? "," : "") << "\n    {\"is_outline\": " << (res.paths[pi].is_outline ? "true" : "false") << ", \"points\": [";
    for (size_t i = 0; i < res.paths[pi].path.poses.size(); ++i) {
      const auto &q = res.paths[pi].path.poses[i].pose.position;
      f << (i ? "," : "") << "[" << q.x << "," << q.y << "]";
    }
    f << "]}";
  }
  f << "\n  ]\n}\n";
}

// One coloured LINE_STRIP per path; marker ns kept from slic3r so RViz configs bind.
static void createMarkers(const coverage_planner::PlanPathResponse &res,
                          visualization_msgs::MarkerArray &arr) {
  std::vector<std_msgs::ColorRGBA> colors;
  auto add = [&](double r, double g, double b) { std_msgs::ColorRGBA c; c.r = r; c.g = g; c.b = b; c.a = 1; colors.push_back(c); };
  add(1, 0, 0); add(0, 1, 0); add(0, 0, 1); add(1, 1, 0); add(1, 0, 1); add(0, 1, 1); add(1, 1, 1);
  uint32_t ci = 0;
  for (const auto &path : res.paths) {
    visualization_msgs::Marker m; m.header.frame_id = "map"; m.ns = "mower_map_service_lines";
    m.id = (int)arr.markers.size(); m.frame_locked = true; m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP; m.color = colors[ci]; m.pose.orientation.w = 1;
    m.scale.x = m.scale.y = m.scale.z = 0.02;
    for (const auto &ps : path.path.poses) {
      geometry_msgs::Point p; p.x = ps.pose.position.x; p.y = ps.pose.position.y; m.points.push_back(p);
    }
    arr.markers.push_back(m);
    ci = (ci + 1) % colors.size();
  }
}

// ---- pipeline --------------------------------------------------------------------------

static bool planPathImpl(coverage_planner::PlanPathRequest &req,
                         coverage_planner::PlanPathResponse &res) {
  PlanConfig cfg = buildConfig(req);
  Field f = buildField(req, cfg);
  if (bg::is_empty(f.mowable)) { ROS_ERROR("coverage_planner: empty mowable area."); return true; }
  ROS_INFO_STREAM("coverage_planner: [1/6] field " << bg::area(f.mowable) << " m^2, "
                  << f.ring_obstacles.size() << " ring obstacle(s), blade " << cfg.blade_diameter
                  << " m (" << (g_tool_width > 0.0 ? "param"
                                : req.distance > 1e-6 ? "request" : "default")
                  << "), overlap " << g_overlap << " -> swath " << cfg.swath_step
                  << " m, clearance " << cfg.clearance
                  << (g_clearance >= 0.0 ? " (param)" : " (request)")
                  << " / obstacle " << cfg.obstacle_clearance
                  << (g_obstacle_clearance >= 0.0 ? " (param) m, " : " m, ")
                  << cfg.n_perimeter_passes
                  << (g_outline_count >= 0 ? " perimeter pass(es) (param)" : " perimeter pass(es)")
                  << ", edge_side=" << cfg.edge_side << ", mow_angle="
                  << cfg.angle_deg << " deg"
                  << (g_optimize_angle ? " (overridden by optimize_sweep_angle)" : "") << ".");

  // Fill starts half a swath inside the innermost loop; no loops -> it rides
  // the boundary clearance itself (see ride_bounds). EMPTY inner means the
  // loops converge past the medial axis and cover the zone alone (narrow
  // strip): no fill -- never fill on top of the loops.
  BMultiPolygon inner = bufferPolygon(f.mowable, -cfg.fill_border());

  std::vector<Pass> loops = loopPasses(f, cfg);
  std::vector<Pass> perimeter, obstacles;
  for (auto &p : loops) {
    if (p.kind == Pass::PERIMETER) perimeter.push_back(std::move(p));
    else obstacles.push_back(std::move(p));
  }
  ROS_INFO_STREAM("coverage_planner: [2/6] perimeter - " << perimeter.size() << " loop-run(s)"
                  << (cfg.n_perimeter_passes > 0
                      ? ", loop insets " + std::to_string(cfg.clearance) + ".."
                        + std::to_string(cfg.clearance + (cfg.n_perimeter_passes - 1) * cfg.swath_step) + " m"
                      : std::string()) << ".");
  ROS_INFO_STREAM("coverage_planner: [3/6] obstacles - " << obstacles.size() << " loop-run(s).");
  std::vector<Pass> fill = fillArea(f, inner, cfg);
  ROS_INFO_STREAM("coverage_planner: [4/6] fill - " << fill.size() << " run(s) ("
                  << (cfg.concentric ? "concentric" : "serpentine")
                  << (bg::is_empty(inner) ? ", loops cover the zone" : "") << ").");

  // Perimeter first; fill + obstacles ordered together so each obstacle is
  // serviced when the mower is already beside it.
  Pt start(0, 0);
  if (!perimeter.empty() && !perimeter.back().pts.empty()) start = perimeter.back().pts.back();
  std::vector<Pass> work;
  work.insert(work.end(), fill.begin(), fill.end());
  work.insert(work.end(), obstacles.begin(), obstacles.end());
  orderByNearest(work, start);
  ROS_INFO_STREAM("coverage_planner: [5/6] sequenced " << work.size()
                  << " fill+obstacle run(s) by nearest-neighbour.");

  std_msgs::Header h; h.stamp = ros::Time::now(); h.frame_id = "map"; h.seq = 0;
  std::vector<const Pass *> order;
  for (const auto &p : perimeter) order.push_back(&p);
  for (const auto &p : work) order.push_back(&p);
  warnObstacleTurns(order, f, cfg);
  double len_perim = 0, len_obst = 0, len_fill = 0;
  int n_perim = 0, n_obst = 0, n_fill = 0;
  for (const Pass *p : order) {
    auto path = emitPath(h, p->pts, p->is_outline);
    if (path.path.poses.empty()) continue;
    double L = pathLength(p->pts);
    if (p->kind == Pass::PERIMETER)          { len_perim += L; ++n_perim; }
    else if (p->kind == Pass::OBSTACLE_RING) { len_obst  += L; ++n_obst;  }
    else                                     { len_fill  += L; ++n_fill;  }
    res.paths.push_back(path);
  }

  if (g_visualize) {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del; del.header.frame_id = "map"; del.ns = "mower_map_service_lines";
    del.id = -1; del.action = visualization_msgs::Marker::DELETEALL; arr.markers.push_back(del);
    createMarkers(res, arr); g_markers.publish(arr);
  }
  dumpPlan(g_dump_file, req, res);
  if (res.paths.empty())
    ROS_WARN("coverage_planner: zone produced no paths (narrower than 2x clearance?).");
  ROS_INFO_STREAM("coverage_planner: [6/6] emitted " << res.paths.size() << " path(s), total "
                  << (len_perim + len_obst + len_fill) << " m   (perimeter " << n_perim << " run/"
                  << len_perim << " m, obstacle " << n_obst << " run/" << len_obst << " m, fill "
                  << n_fill << " run/" << len_fill << " m).");
  return true;
}

// Never let an exception reach ROS; empty response lets mower_logic fall back.
static bool planPath(coverage_planner::PlanPathRequest &req,
                     coverage_planner::PlanPathResponse &res) {
  try { return planPathImpl(req, res); }
  catch (const std::exception &e) { ROS_ERROR_STREAM("coverage_planner threw: " << e.what()); }
  catch (...) { ROS_ERROR("coverage_planner threw unknown exception."); }
  res.paths.clear(); return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "coverage_planner");
  ros::NodeHandle n, pn("~");
  g_visualize = pn.param("visualize_plan", true);
  g_lane_skip = std::max(1, pn.param("lane_skip", 2));
  g_optimize_angle = pn.param("optimize_sweep_angle", true);   // true replaces req.angle field-wide
  g_blade_off_x = pn.param("blade_offset_x", 0.0);
  g_blade_off_y = pn.param("blade_offset_y", 0.0);
  g_tool_width = pn.param("tool_width", 0.22);                   // <= 0 follows the request
  g_overlap = std::min(0.9, std::max(0.0, pn.param("overlap", 0.35)));
  g_clearance = pn.param("clearance", 0.04);                     // < 0 follows the request
  g_obstacle_clearance = pn.param("obstacle_clearance", -1.0);   // < 0 follows clearance
  g_outline_count = pn.param("outline_count", 3);               // < 0 follows the request
  g_fill_mode = pn.param<std::string>("fill_mode", std::string("request"));  // request|serpentine|concentric
  g_path_spacing = std::min(0.5, std::max(0.02, pn.param("path_spacing", 0.1)));
  g_dump_file = pn.param<std::string>("debug_dump_file", std::string());
  g_body_width = pn.param("body_width", 0.39);
  g_body_length = pn.param("body_length", 0.58);
  g_min_turn_radius = pn.param("min_turn_radius", 0.0);
  g_loop_transition = std::max(1.0, pn.param("loop_transition", 6.0));
  g_edge_side = pn.param<std::string>("edge_side", std::string("right"));
  g_axle_from_rear = pn.param("axle_from_rear", 0.11);
  if (g_visualize)
    g_markers = n.advertise<visualization_msgs::MarkerArray>("slic3r_coverage_planner/path_marker_array", 100, true);
  // Registered under the slic3r name: drop-in for slic3r_coverage_planner.
  ros::ServiceServer srv = n.advertiseService("slic3r_coverage_planner/plan_path", planPath);
  ROS_INFO("coverage_planner v2 ready.");
  ros::spin();
  return 0;
}