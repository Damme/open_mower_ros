#pragma once
// geom.hpp - pure geometry helpers for the coverage planner. No ROS, no planner
// judgement; Boost.Geometry wrappers + polyline math.

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>

namespace bg = boost::geometry;

namespace geom {

using BPoint = bg::model::d2::point_xy<double>;
using BPolygon = bg::model::polygon<BPoint, /*Clockwise=*/false, /*Closed=*/true>;
using BMultiPolygon = bg::model::multi_polygon<BPolygon>;
using BLine = bg::model::linestring<BPoint>;
using BMultiLine = bg::model::multi_linestring<BLine>;

struct Pt {
  double x = 0.0, y = 0.0;
  Pt() = default;
  Pt(double X, double Y) : x(X), y(Y) {}
};

inline double dist(const Pt &a, const Pt &b) { return std::hypot(a.x - b.x, a.y - b.y); }

inline double pathLength(const std::vector<Pt> &p) {
  double L = 0.0;
  for (size_t i = 1; i < p.size(); ++i) L += dist(p[i - 1], p[i]);
  return L;
}

static const std::size_t kBufferPointsPerCircle = 8;  // buffer corner facets (8 = ~45 deg)

// ---- Boost wrappers --------------------------------------------------------

inline BPolygon makePolygon(const std::vector<Pt> &ring) {
  BPolygon poly;
  if (ring.size() < 3) return poly;                    // degenerate -> empty
  for (const auto &p : ring) bg::append(poly.outer(), BPoint(p.x, p.y));
  if (!bg::equals(poly.outer().front(), poly.outer().back()))
    bg::append(poly.outer(), poly.outer().front());    // close
  bg::correct(poly);
  return poly;
}

// Offset by d (negative shrinks). Boost buffer returns EMPTY at d == 0, so
// treat ~0 as identity -- never use buffer(0) as a repair or a no-op.
inline BMultiPolygon bufferPolygon(const BMultiPolygon &in, double d) {
  BMultiPolygon out;
  if (bg::is_empty(in)) return out;
  if (std::fabs(d) < 1e-12) return in;                 // identity, see above
  bg::strategy::buffer::distance_symmetric<double> ds(d);
  bg::strategy::buffer::join_round js(kBufferPointsPerCircle);
  bg::strategy::buffer::end_round es(kBufferPointsPerCircle);
  bg::strategy::buffer::point_circle cs(kBufferPointsPerCircle);
  bg::strategy::buffer::side_straight ss;
  try { bg::buffer(in, out, ds, ss, js, es, cs); } catch (...) { out.clear(); }
  return out;
}
inline BMultiPolygon bufferPolygon(const BPolygon &in, double d) {
  BMultiPolygon m; m.push_back(in); return bufferPolygon(m, d);
}

inline BMultiPolygon toMulti(const BPolygon &p) {
  BMultiPolygon m; if (!bg::is_empty(p)) m.push_back(p); return m;
}

inline BPolygon largestComponent(const BMultiPolygon &m) {
  BPolygon best; double best_a = -1.0;
  for (const auto &p : m) { double a = bg::area(p); if (a > best_a) { best_a = a; best = p; } }
  return best;
}

inline bool multiContains(const BMultiPolygon &m, const BPoint &pt) {
  for (const auto &p : m) if (bg::covered_by(pt, p)) return true;   // boundary counts
  return false;
}

inline Pt centroidOf(const BMultiPolygon &m) {
  BPoint c(0, 0); try { bg::centroid(m, c); } catch (...) {}
  return Pt(bg::get<0>(c), bg::get<1>(c));
}

inline void boundsOf(const BMultiPolygon &m, double &minx, double &miny,
                     double &maxx, double &maxy) {
  minx = miny = maxx = maxy = 0.0;
  if (bg::is_empty(m)) return;
  try {
    bg::model::box<BPoint> box; bg::envelope(m, box);
    minx = bg::get<bg::min_corner, 0>(box); miny = bg::get<bg::min_corner, 1>(box);
    maxx = bg::get<bg::max_corner, 0>(box); maxy = bg::get<bg::max_corner, 1>(box);
  } catch (...) {}
}

// Rotate about origin, deg CCW positive (Boost's transformer is CW, hence -deg).
inline BMultiPolygon rotateMulti(const BMultiPolygon &m, double deg, const Pt &origin) {
  bg::strategy::transform::translate_transformer<double, 2, 2> to_o(-origin.x, -origin.y);
  bg::strategy::transform::rotate_transformer<bg::degree, double, 2, 2> rot(-deg);
  bg::strategy::transform::translate_transformer<double, 2, 2> back(origin.x, origin.y);
  BMultiPolygon t1, t2, out;
  bg::transform(m, t1, to_o); bg::transform(t1, t2, rot); bg::transform(t2, out, back);
  return out;
}

inline std::vector<Pt> outerRing(const BPolygon &p) {
  std::vector<Pt> out;
  for (const auto &c : p.outer()) out.emplace_back(bg::get<0>(c), bg::get<1>(c));
  return out;
}

inline std::vector<std::vector<Pt>> innerRings(const BPolygon &p) {
  std::vector<std::vector<Pt>> out;
  for (const auto &r : p.inners()) {
    std::vector<Pt> ring;
    for (const auto &c : r) ring.emplace_back(bg::get<0>(c), bg::get<1>(c));
    if (ring.size() >= 3) out.push_back(std::move(ring));
  }
  return out;
}

// ---- ring winding / repair -------------------------------------------------

inline bool ringCCW(const std::vector<Pt> &c) {
  if (c.size() < 3) return true;
  size_t n = c.size();
  if (dist(c.front(), c.back()) < 1e-12) --n;          // ignore closing vertex
  double s = 0.0;
  for (size_t i = 0; i < n; ++i) { const Pt &a = c[i], &b = c[(i + 1) % n]; s += a.x * b.y - b.x * a.y; }
  return s > 0.0;                                      // shoelace
}

inline std::vector<Pt> orientRing(std::vector<Pt> c, bool want_ccw) {
  if (c.size() >= 3 && ringCCW(c) != want_ccw) std::reverse(c.begin(), c.end());
  return c;
}

// Make an outline valid: bg::correct, else morphological close (+eps/-eps
// re-traces the drawn figure as a valid union). Shapely's buffer(0) repair does
// NOT work in Boost (empty output). Last resort passes the input through --
// every downstream Boost op is try/catch wrapped.
inline BMultiPolygon sanitize(const BPolygon &in) {
  BPolygon p = in;
  try { bg::correct(p); } catch (...) {}
  bool ok = false; try { ok = bg::is_valid(p); } catch (...) { ok = false; }
  if (ok) return toMulti(p);
  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  for (const auto &pt : p.outer()) {
    double x = bg::get<0>(pt), y = bg::get<1>(pt);
    minx = std::min(minx, x); maxx = std::max(maxx, x);
    miny = std::min(miny, y); maxy = std::max(maxy, y);
  }
  double eps = std::min(0.05, std::max(1e-3, std::hypot(maxx - minx, maxy - miny) * 1e-3));
  BMultiPolygon closed = bufferPolygon(bufferPolygon(toMulti(p), eps), -eps);
  return bg::is_empty(closed) ? toMulti(p) : closed;
}

// ---- scan-line clip ----------------------------------------------------------

inline std::vector<std::vector<Pt>> lineSegments(const BMultiLine &ml) {
  std::vector<std::vector<Pt>> out;
  for (const auto &ls : ml) {
    std::vector<Pt> seg; for (const auto &c : ls) seg.emplace_back(bg::get<0>(c), bg::get<1>(c));
    if (seg.size() >= 2) out.push_back(seg);
  }
  return out;
}

inline std::vector<std::vector<Pt>> scanLineClip(const BMultiPolygon &poly, double y,
                                                 double minx, double maxx) {
  BLine line; bg::append(line, BPoint(minx - 1.0, y)); bg::append(line, BPoint(maxx + 1.0, y));
  BMultiLine clipped; try { bg::intersection(line, poly, clipped); } catch (...) { clipped.clear(); }
  return lineSegments(clipped);
}

// Break a path where it enters `region`, keep the outside pieces. Empty region
// -> unchanged; a FAILED difference -> nothing (fail safe, never cross a keep-out).
inline std::vector<std::vector<Pt>> clipPathOutside(const std::vector<Pt> &path,
                                                    const BMultiPolygon &region) {
  std::vector<std::vector<Pt>> out;
  if (path.size() < 2) { if (!path.empty()) out.push_back(path); return out; }
  if (bg::is_empty(region)) { out.push_back(path); return out; }
  BLine line;
  for (const auto &p : path) line.push_back(BPoint(p.x, p.y));
  BMultiLine outside;
  try { bg::difference(line, region, outside); } catch (...) { return out; }
  return lineSegments(outside);
}

// ---- polyline utilities ------------------------------------------------------

// Even spacing; keeps first and last input point.
inline std::vector<Pt> resample(const std::vector<Pt> &in, double spacing) {
  std::vector<Pt> out;
  if (in.size() < 2) return in;
  out.push_back(in.front()); double carry = 0.0;
  for (size_t i = 1; i < in.size(); ++i) {
    Pt a = in[i - 1], b = in[i]; double seg = dist(a, b); if (seg < 1e-9) continue;
    Pt dir((b.x - a.x) / seg, (b.y - a.y) / seg); double pos = spacing - carry;
    while (pos < seg) { out.emplace_back(a.x + dir.x * pos, a.y + dir.y * pos); pos += spacing; }
    carry = seg - (pos - spacing);                     // carry remainder over the vertex
  }
  if (dist(out.back(), in.back()) > 1e-6) out.push_back(in.back());
  return out;
}

inline std::vector<Pt> cubicBezier(const Pt &p0, const Pt &p1, const Pt &p2,
                                   const Pt &p3, int n = 16) {
  std::vector<Pt> out; out.reserve(n);
  for (int i = 0; i < n; ++i) {
    double t = (n > 1) ? (double)i / (n - 1) : 0.0, u = 1.0 - t;
    double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
    out.emplace_back(b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
                     b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y);
  }
  return out;
}

// Tangent-matched turn from a (heading ah->a) into b (heading b->bh); r = how
// far the control points sit along the tangents (bigger = wider turn).
inline std::vector<Pt> makeTurn(const Pt &a, const Pt &ah, const Pt &b, const Pt &bh, double r) {
  Pt din(a.x - ah.x, a.y - ah.y); double l1 = std::hypot(din.x, din.y);
  if (l1 > 1e-9) { din.x /= l1; din.y /= l1; }
  Pt dout(bh.x - b.x, bh.y - b.y); double l2 = std::hypot(dout.x, dout.y);
  if (l2 > 1e-9) { dout.x /= l2; dout.y /= l2; }
  Pt c1(a.x + din.x * r, a.y + din.y * r), c2(b.x - dout.x * r, b.y - dout.y * r);
  return cubicBezier(a, c1, c2, b, 12);
}

// Sampled centerline fully covered by `region` (no body-footprint check).
inline bool pathInside(const std::vector<Pt> &line, const BMultiPolygon &region,
                       double spacing = 0.05) {
  if (bg::is_empty(region)) return false;
  for (const auto &p : resample(line, spacing))
    if (!multiContains(region, BPoint(p.x, p.y))) return false;
  return true;
}

// ---- stacked loops -----------------------------------------------------------

// Drive each closed ring (in given order) as one full loop, then blend into the
// next: enter it `fwd` ring-gaps forward of the nearest point via a shallow
// tangent-matched bezier (control = chord/3). If `blend_ok` is given, the entry
// distance is halved until the blend passes it; the final one-gap hop at the
// nearest point stays inside the ring band, so it is accepted as the fallback.
inline std::vector<Pt> stackLoops(const std::vector<std::vector<Pt>> &rings, double spacing,
                                  double fwd = 6.0,
                                  const std::function<bool(const std::vector<Pt> &)> &blend_ok = {}) {
  std::vector<Pt> out;
  for (const auto &ring : rings) {
    std::vector<Pt> loop = resample(ring, spacing);
    if (loop.size() >= 2 && dist(loop.front(), loop.back()) < 1e-6) loop.pop_back();
    int N = (int)loop.size();
    if (N < 2) continue;
    if (out.empty()) {                                       // first loop: full, no blend
      for (int i = 0; i <= N; ++i) out.push_back(loop[i % N]);
      continue;
    }
    Pt prev = out.back();
    int s0 = 0; double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < N; ++i) { double dd = dist(prev, loop[i]); if (dd < best) { best = dd; s0 = i; } }
    double gap = std::max(best, 1e-6);                       // ring-to-ring distance here
    Pt tin(prev.x - out[out.size() - 2].x, prev.y - out[out.size() - 2].y);
    double li = std::hypot(tin.x, tin.y); if (li > 1e-9) { tin.x /= li; tin.y /= li; }
    int adv = std::max(1, (int)std::round(fwd * gap / spacing));
    adv = std::min(adv, std::max(1, N / 3));                 // small ring: cap the entry
    std::vector<Pt> blend; int s = s0;
    for (;; adv /= 2) {                                      // halve the entry until it fits
      if (adv < 1) adv = 1;
      s = (s0 + adv) % N;
      Pt nx = loop[(s + 1) % N];
      Pt tout(nx.x - loop[s].x, nx.y - loop[s].y);
      double lo = std::hypot(tout.x, tout.y); if (lo > 1e-9) { tout.x /= lo; tout.y /= lo; }
      double chord = dist(prev, loop[s]), rr = chord / 3.0;
      Pt c1(prev.x + tin.x * rr, prev.y + tin.y * rr), c2(loop[s].x - tout.x * rr, loop[s].y - tout.y * rr);
      blend = cubicBezier(prev, c1, c2, loop[s], std::max(8, (int)(chord / spacing) + 2));
      if (adv == 1 || !blend_ok || blend_ok(blend)) break;   // adv 1 = one-gap hop, safe
    }
    for (size_t j = 1; j < blend.size(); ++j) out.push_back(blend[j]);
    for (int i = 1; i <= N; ++i) out.push_back(loop[(s + i) % N]);   // full loop from entry
  }
  return out;
}

// ---- region metrics ----------------------------------------------------------

// Inscribed-disk diameter via binary search on a negative buffer.
inline double inscribedWidth(const BMultiPolygon &cell) {
  if (bg::is_empty(cell)) return 0.0;
  double minx, miny, maxx, maxy; boundsOf(cell, minx, miny, maxx, maxy);
  double hi = 0.5 * std::min(maxx - minx, maxy - miny), lo = 0.0, r = 0.0;
  for (int it = 0; it < 24; ++it) {
    double mid = 0.5 * (lo + hi);
    if (!bg::is_empty(bufferPolygon(cell, -mid))) { r = mid; lo = mid; } else hi = mid;
  }
  return 2.0 * r;
}

// Major-axis angle [deg] via PCA.
inline double principalAxisDeg(const std::vector<Pt> &pts) {
  if (pts.size() < 2) return 0.0;
  double mx = 0, my = 0; for (const auto &p : pts) { mx += p.x; my += p.y; }
  mx /= pts.size(); my /= pts.size();
  double sxx = 0, sxy = 0, syy = 0;
  for (const auto &p : pts) { double dx = p.x - mx, dy = p.y - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
  return 0.5 * std::atan2(2 * sxy, sxx - syy) * 180.0 / M_PI;
}

// ---- offset cutting deck -------------------------------------------------------

// base = blade - R(heading)*offset; offset is body frame (+x fwd, +y left).
inline Pt bladeToBase(const Pt &blade, double h, double ox, double oy) {
  double c = std::cos(h), s = std::sin(h);
  return Pt(blade.x - (c * ox - s * oy), blade.y - (s * ox + c * oy));
}

inline std::vector<Pt> bladePathToBase(const std::vector<Pt> &blade, double ox, double oy) {
  if (ox == 0.0 && oy == 0.0) return blade;
  size_t n = blade.size(); if (n < 2) return blade;
  std::vector<Pt> out(n);
  for (size_t i = 0; i < n; ++i) {
    const Pt &a = blade[i == 0 ? 0 : i - 1], &b = blade[i + 1 < n ? i + 1 : n - 1];
    out[i] = bladeToBase(blade[i], std::atan2(b.y - a.y, b.x - a.x), ox, oy);  // heading by difference
  }
  return out;
}

}  // namespace geom