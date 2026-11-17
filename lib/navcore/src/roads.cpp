#include "roads.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <new>

#include "pbf.h"

namespace nav {

using pbf::Cursor;

namespace {

constexpr char kMagic[4] = {'G', 'N', 'R', '1'};
constexpr size_t kHeaderSize = 32;
constexpr size_t kNodeSize = 8, kAdjSize = 12, kEdgeSize = 22, kGeomSize = 4, kBoundSize = 4;
constexpr size_t kMaxPoints = 65000;
constexpr int kMaxEdgeLen = 30000;
// Tolerances in units of a 4096-extent tile (~0.6 m at the equator, ~0.45 m at 40 deg).
constexpr double kSnapUnits = 10;    // dangling line end joins a line this close (T junction gaps)
constexpr double kBorderUnits = 12;  // nodes on a tile border join the neighbour's node this close
constexpr uint8_t kAdjNoDrive = 1, kAdjReversed = 2;

bool fits(int w, size_t n) { return w > 0 && size_t(w) < n; }

// ---------------------------------------------------------------- MVT parsing

struct Val {
    size_t s = 0, l = 0;  // string value
    bool isStr = false;
    int64_t i = 0;
    bool isInt = false;
};

bool spanIs(ByteSource &src, const Val &v, const char *str) {
    char buf[24];
    const size_t n = strlen(str);
    if (!v.isStr || v.l != n || n >= sizeof buf) return false;
    return src.read(v.s, reinterpret_cast<uint8_t *>(buf), n) && memcmp(buf, str, n) == 0;
}

bool spanEq(ByteSource &src, size_t s, size_t l, const char *str) {
    Val v;
    v.s = s;
    v.l = l;
    v.isStr = true;
    return spanIs(src, v, str);
}

int roadClass(ByteSource &src, const Val &v) {
    static const char *const kNames[kRoadClassCount] = {"motorway", "trunk",   "primary", "secondary", "tertiary",
                                                        "minor",    "service", "track",   "path"};
    for (int c = 0; c < kRoadClassCount; ++c)
        if (spanIs(src, v, kNames[c])) return c;
    return -1;
}

bool decodeLines(ByteSource &src, size_t start, size_t len, const RoadLine &proto, RoadLines &out) {
    Cursor g(src, start, start + len);
    int64_t x = 0, y = 0;
    RoadLine *cur = nullptr;
    while (!g.done()) {
        uint64_t cmd;
        if (!g.varint(cmd)) return false;
        const uint32_t id = uint32_t(cmd & 7), count = uint32_t(cmd >> 3);
        if (id == 7) continue;
        if (id != 1 && id != 2) return false;
        for (uint32_t i = 0; i < count; ++i) {
            uint64_t zx, zy;
            if (!g.varint(zx) || !g.varint(zy)) return false;
            x += int64_t((zx >> 1) ^ (~(zx & 1) + 1));
            y += int64_t((zy >> 1) ^ (~(zy & 1) + 1));
            if (id == 1 || !cur) {
                out.lines.push_back(proto);
                cur = &out.lines.back();
                cur->start = uint32_t(out.xy.size() / 2);
            }
            if (cur->count == 0xFFFF) return false;
            out.xy.push_back(int16_t(x < -32768 ? -32768 : x > 32767 ? 32767 : x));
            out.xy.push_back(int16_t(y < -32768 ? -32768 : y > 32767 ? 32767 : y));
            ++cur->count;
        }
    }
    return true;
}

int parseLayer(ByteSource &src, size_t start, size_t len, RoadLines &out, uint16_t &extentOut) {
    {
        Cursor c(src, start, start + len);
        uint32_t num, wire;
        bool named = false;
        while (!c.done() && !named) {
            if (!c.field(num, wire)) return -1;
            size_t s, l;
            if (num == 1 && wire == 2) {
                if (!c.lenDelim(s, l)) return -1;
                if (!spanEq(src, s, l, "transportation")) return 0;
                named = true;
            } else if (!c.skipValue(wire)) {
                return -1;
            }
        }
        if (!named) return 0;
    }
    int kClass = -1, kOneway = -1, kBrunnel = -1, kLayer = -1, keyIdx = 0;
    uint64_t extent = 4096;
    std::vector<Val> values;
    {
        Cursor c(src, start, start + len);
        uint32_t num, wire;
        while (!c.done()) {
            if (!c.field(num, wire)) return -1;
            size_t s, l;
            if (num == 3 && wire == 2) {
                if (!c.lenDelim(s, l)) return -1;
                if (spanEq(src, s, l, "class")) kClass = keyIdx;
                else if (spanEq(src, s, l, "oneway")) kOneway = keyIdx;
                else if (spanEq(src, s, l, "brunnel")) kBrunnel = keyIdx;
                else if (spanEq(src, s, l, "layer")) kLayer = keyIdx;
                ++keyIdx;
            } else if (num == 4 && wire == 2) {
                if (!c.lenDelim(s, l)) return -1;
                Val v;
                Cursor vc(src, s, s + l);
                uint32_t vn, vw;
                while (!vc.done()) {
                    if (!vc.field(vn, vw)) return -1;
                    uint64_t u;
                    size_t ss, sl;
                    if (vn == 1 && vw == 2) {
                        if (!vc.lenDelim(ss, sl)) return -1;
                        v.s = ss;
                        v.l = sl;
                        v.isStr = true;
                    } else if ((vn == 4 || vn == 5) && vw == 0) {
                        if (!vc.varint(u)) return -1;
                        v.i = int64_t(u);
                        v.isInt = true;
                    } else if (vn == 6 && vw == 0) {
                        if (!vc.varint(u)) return -1;
                        v.i = int64_t((u >> 1) ^ (~(u & 1) + 1));
                        v.isInt = true;
                    } else if (!vc.skipValue(vw)) {
                        return -1;
                    }
                }
                values.push_back(v);
            } else if (num == 5 && wire == 0) {
                if (!c.varint(extent)) return -1;
            } else if (!c.skipValue(wire)) {
                return -1;
            }
        }
    }
    if (extent == 0 || extent > 16384) return -1;
    extentOut = uint16_t(extent);
    if (kClass < 0) return 0;

    Cursor c(src, start, start + len);
    uint32_t num, wire;
    while (!c.done()) {
        if (!c.field(num, wire)) return -1;
        if (num != 2 || wire != 2) {
            if (!c.skipValue(wire)) return -1;
            continue;
        }
        size_t fs, fl;
        if (!c.lenDelim(fs, fl)) return -1;
        Cursor f(src, fs, fs + fl);
        uint64_t type = 0;
        size_t ts = 0, tl = 0, gs = 0, gl = 0;
        uint32_t fn, fw;
        while (!f.done()) {
            if (!f.field(fn, fw)) return -1;
            if (fn == 2 && fw == 2) {
                if (!f.lenDelim(ts, tl)) return -1;
            } else if (fn == 3 && fw == 0) {
                if (!f.varint(type)) return -1;
            } else if (fn == 4 && fw == 2) {
                if (!f.lenDelim(gs, gl)) return -1;
            } else if (!f.skipValue(fw)) {
                return -1;
            }
        }
        if (type != 2 || !gl) continue;
        RoadLine proto;
        int cls = -1;
        Cursor t(src, ts, ts + tl);
        while (!t.done()) {
            uint64_t k, v;
            if (!t.varint(k) || !t.varint(v)) return -1;
            if (v >= values.size()) continue;
            const Val &val = values[v];
            if (int(k) == kClass) cls = roadClass(src, val);
            else if (int(k) == kOneway && val.isInt) proto.flags |= val.i > 0 ? kRoadOneway : val.i < 0 ? kRoadOnewayRev : 0;
            else if (int(k) == kBrunnel && (spanIs(src, val, "bridge") || spanIs(src, val, "tunnel")))
                proto.flags |= kRoadBrunnel;
            else if (int(k) == kLayer && val.isInt) proto.layer = int8_t(std::max<int64_t>(-8, std::min<int64_t>(8, val.i)));
        }
        if (cls < 0) continue;
        proto.cls = uint8_t(cls);
        if (!decodeLines(src, gs, gl, proto, out)) return -1;
    }
    return 1;
}

// ---------------------------------------------------------------- graph building

struct P16 {
    int16_t x, y;
    bool operator==(const P16 &o) const { return x == o.x && y == o.y; }
    bool operator!=(const P16 &o) const { return !(*this == o); }
};

struct Piece {
    uint32_t start = 0, count = 0;
    uint8_t cls = 0, flags = 0;
    int8_t layer = 0;
};

struct Split {
    uint32_t seg;
    float t;
    P16 p;
};

uint32_t key(P16 p) { return (uint32_t(uint16_t(p.x)) << 16) | uint16_t(p.y); }

bool joinable(const Piece &a, const Piece &b) {
    if ((a.cls == kRoadMotorway) != (b.cls == kRoadMotorway)) return false;
    if (bool(a.flags & kRoadBrunnel) != bool(b.flags & kRoadBrunnel)) return false;
    return a.layer == b.layer;
}

bool clipSeg(double x0, double y0, double x1, double y1, double e, double &t0, double &t1) {
    t0 = 0;
    t1 = 1;
    const double dx = x1 - x0, dy = y1 - y0;
    const double p[4] = {-dx, dx, -dy, dy}, q[4] = {x0, e - x0, y0, e - y0};
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0) {
            if (q[i] < 0) return false;
            continue;
        }
        const double r = q[i] / p[i];
        if (p[i] < 0) {
            if (r > t1) return false;
            if (r > t0) t0 = r;
        } else {
            if (r < t0) return false;
            if (r < t1) t1 = r;
        }
    }
    return true;
}

P16 roundPt(double x, double y, int e) {
    long ix = lround(x), iy = lround(y);
    ix = ix < 0 ? 0 : ix > e ? e : ix;
    iy = iy < 0 ? 0 : iy > e ? e : iy;
    return {int16_t(ix), int16_t(iy)};
}

double projT(double px, double py, P16 a, P16 b, double &d2) {
    const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((px - a.x) * dx + (py - a.y) * dy) / l2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const double cx = a.x + t * dx - px, cy = a.y + t * dy - py;
    d2 = cx * cx + cy * cy;
    return t;
}

struct Builder {
    int e = 4096;
    std::vector<P16> pts;
    std::vector<uint8_t> isNode;
    std::vector<uint16_t> pieceOf;
    std::vector<Piece> pieces;

    // Segment grid; segment i runs from pts[i] to pts[i + 1] of the same piece.
    int cell = 128, gn = 0;
    std::vector<uint32_t> cellStart;
    std::vector<uint16_t> cellItems;

    bool segValid(uint32_t i) const { return i + 1 < pts.size() && pieceOf[i] == pieceOf[i + 1]; }
    int cellOf(double v) const {
        int c = int(floor(v / cell));
        return c < 0 ? 0 : c >= gn ? gn - 1 : c;
    }

    void clip(RoadLines &rl) {
        for (const auto &ln : rl.lines) {
            const int16_t *lp = rl.pts(ln);
            Piece cur;
            bool open = false;
            auto close = [&]() {
                if (open && cur.count >= 2) pieces.push_back(cur);
                else if (open) pts.resize(cur.start);
                open = false;
            };
            const size_t np = ln.count;
            for (size_t i = 0; i + 1 < np; ++i) {
                const double x0 = lp[2 * i], y0 = lp[2 * i + 1], x1 = lp[2 * i + 2], y1 = lp[2 * i + 3];
                double t0, t1;
                if (!clipSeg(x0, y0, x1, y1, e, t0, t1)) {
                    close();
                    continue;
                }
                const P16 a = roundPt(x0 + t0 * (x1 - x0), y0 + t0 * (y1 - y0), e);
                const P16 b = roundPt(x0 + t1 * (x1 - x0), y0 + t1 * (y1 - y0), e);
                if (!open || t0 > 0) {
                    close();
                    cur = Piece();
                    cur.start = uint32_t(pts.size());
                    cur.cls = ln.cls;
                    cur.flags = ln.flags;
                    cur.layer = ln.layer;
                    open = true;
                    pts.push_back(a);
                    cur.count = 1;
                }
                if (b != pts.back()) {
                    pts.push_back(b);
                    ++cur.count;
                }
                if (t1 < 1) close();
            }
            close();
        }
        std::vector<RoadLine>().swap(rl.lines);
        std::vector<int16_t>().swap(rl.xy);
        isNode.assign(pts.size(), 0);
        pieceOf.assign(pts.size(), 0);
        for (size_t p = 0; p < pieces.size(); ++p) {
            for (uint32_t k = 0; k < pieces[p].count; ++k) pieceOf[pieces[p].start + k] = uint16_t(p);
            isNode[pieces[p].start] = isNode[pieces[p].start + pieces[p].count - 1] = 1;
        }
    }

    void buildGrid(double margin) {
        gn = e / cell + 1;
        cellStart.assign(size_t(gn) * gn + 1, 0);
        auto forCells = [&](uint32_t i, auto fn) {
            const P16 a = pts[i], b = pts[i + 1];
            const int cx0 = cellOf(std::min(a.x, b.x) - margin), cx1 = cellOf(std::max(a.x, b.x) + margin);
            const int cy0 = cellOf(std::min(a.y, b.y) - margin), cy1 = cellOf(std::max(a.y, b.y) + margin);
            for (int cy = cy0; cy <= cy1; ++cy)
                for (int cx = cx0; cx <= cx1; ++cx) fn(size_t(cy) * gn + cx);
        };
        for (uint32_t i = 0; i < pts.size(); ++i)
            if (segValid(i)) forCells(i, [&](size_t c) { ++cellStart[c + 1]; });
        for (size_t c = 1; c < cellStart.size(); ++c) cellStart[c] += cellStart[c - 1];
        cellItems.assign(cellStart.back(), 0);
        std::vector<uint32_t> fill(cellStart.begin(), cellStart.end() - 1);
        for (uint32_t i = 0; i < pts.size(); ++i)
            if (segValid(i)) forCells(i, [&](size_t c) { cellItems[fill[c]++] = uint16_t(i); });
    }

    void freeGrid() {
        std::vector<uint32_t>().swap(cellStart);
        std::vector<uint16_t>().swap(cellItems);
    }

    // Dangling line ends that stop just short of (or just past) another line are joined to it.
    void snapEnds(std::vector<Split> &splits) {
        const double tol = kSnapUnits * e / 4096.0;
        std::vector<uint32_t> keys(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) keys[i] = key(pts[i]);
        std::sort(keys.begin(), keys.end());
        buildGrid(tol);
        for (size_t p = 0; p < pieces.size(); ++p) {
            for (int end = 0; end < 2; ++end) {
                const uint32_t ei = end ? pieces[p].start + pieces[p].count - 1 : pieces[p].start;
                const P16 q = pts[ei];
                if (q.x == 0 || q.y == 0 || q.x == e || q.y == e) continue;
                const auto range = std::equal_range(keys.begin(), keys.end(), key(q));
                if (range.second - range.first > 1) continue;  // already shares a point with another line
                const int c = cellOf(q.y) * gn + cellOf(q.x);
                double best = tol * tol;
                int bestSeg = -1;
                double bestT = 0;
                for (uint32_t k = cellStart[c]; k < cellStart[c + 1]; ++k) {
                    const uint32_t s = cellItems[k];
                    if (pieceOf[s] == p || !joinable(pieces[p], pieces[pieceOf[s]])) continue;
                    double d2;
                    const double t = projT(q.x, q.y, pts[s], pts[s + 1], d2);
                    if (d2 <= best) {
                        best = d2;
                        bestSeg = int(s);
                        bestT = t;
                    }
                }
                if (bestSeg < 0) continue;
                const P16 a = pts[bestSeg], b = pts[bestSeg + 1];
                P16 m = roundPt(a.x + bestT * (b.x - a.x), a.y + bestT * (b.y - a.y), e);
                auto near = [&](P16 v) { return abs(v.x - m.x) <= 1 && abs(v.y - m.y) <= 1; };
                if (near(a)) m = a;
                else if (near(b)) m = b;
                else splits.push_back({uint32_t(bestSeg), float(bestT), m});
                pts[ei] = m;
            }
        }
        freeGrid();
    }

    // Lines that cross without sharing a point are split at the crossing (unless one is a bridge/tunnel).
    void findCrossings(std::vector<Split> &splits) {
        buildGrid(0);
        for (int c = 0; c < gn * gn; ++c) {
            const int ccx = c % gn, ccy = c / gn;
            for (uint32_t ia = cellStart[c]; ia < cellStart[c + 1]; ++ia) {
                const uint32_t i = cellItems[ia];
                for (uint32_t ib = ia + 1; ib < cellStart[c + 1]; ++ib) {
                    const uint32_t j = cellItems[ib];
                    if (pieceOf[i] == pieceOf[j] && (i + 1 == j || j + 1 == i)) continue;
                    if (!joinable(pieces[pieceOf[i]], pieces[pieceOf[j]])) continue;
                    const P16 A = pts[i], B = pts[i + 1], C = pts[j], D = pts[j + 1];
                    const double rx = B.x - A.x, ry = B.y - A.y, sx = D.x - C.x, sy = D.y - C.y;
                    const double den = rx * sy - ry * sx;
                    if (den == 0) continue;
                    const double qx = C.x - A.x, qy = C.y - A.y;
                    const double t = (qx * sy - qy * sx) / den, u = (qx * ry - qy * rx) / den;
                    if (t < 0 || t > 1 || u < 0 || u > 1) continue;
                    const double X = A.x + t * rx, Y = A.y + t * ry;
                    if (cellOf(X) != ccx || cellOf(Y) != ccy) continue;  // counted in another cell
                    const P16 m = roundPt(X, Y, e);
                    if (m != A && m != B) splits.push_back({i, float(t), m});
                    if (m != C && m != D) splits.push_back({j, float(u), m});
                }
            }
        }
        freeGrid();
    }

    void applySplits(std::vector<Split> &splits) {
        std::sort(splits.begin(), splits.end(),
                  [](const Split &a, const Split &b) { return a.seg != b.seg ? a.seg < b.seg : a.t < b.t; });
        std::vector<P16> np;
        std::vector<uint8_t> nn;
        np.reserve(pts.size() + splits.size());
        nn.reserve(pts.size() + splits.size());
        size_t si = 0;
        for (auto &pc : pieces) {
            const uint32_t start = uint32_t(np.size());
            auto push = [&](P16 p, uint8_t node) {
                if (np.size() > start && np.back() == p) nn.back() |= node;
                else {
                    np.push_back(p);
                    nn.push_back(node);
                }
            };
            for (uint32_t k = 0; k < pc.count; ++k) {
                const uint32_t i = pc.start + k;
                push(pts[i], isNode[i]);
                while (si < splits.size() && splits[si].seg < i) ++si;
                while (si < splits.size() && splits[si].seg == i) push(splits[si++].p, 1);
            }
            pc.start = start;
            pc.count = uint32_t(np.size()) - start;
            nn[start] = nn.back() = 1;
        }
        pts.swap(np);
        isNode.swap(nn);
        std::vector<Split>().swap(splits);
        pieceOf.assign(pts.size(), 0);
        for (size_t p = 0; p < pieces.size(); ++p)
            for (uint32_t k = 0; k < pieces[p].count; ++k) pieceOf[pieces[p].start + k] = uint16_t(p);
    }
};

class BufWriter {
public:
    explicit BufWriter(ByteSink &s) : s_(s) {}
    ~BufWriter() { flush(); }
    bool ok() const { return ok_; }
    void u8(uint8_t v) {
        if (n_ == sizeof buf_) flush();
        buf_[n_++] = v;
    }
    void u16(uint16_t v) {
        u8(uint8_t(v));
        u8(uint8_t(v >> 8));
    }
    void u32(uint32_t v) {
        u16(uint16_t(v));
        u16(uint16_t(v >> 16));
    }
    void raw(const char *p, size_t n) {
        for (size_t i = 0; i < n; ++i) u8(uint8_t(p[i]));
    }
    bool flush() {
        if (n_ && ok_) ok_ = s_.write(buf_, n_);
        n_ = 0;
        return ok_;
    }

private:
    ByteSink &s_;
    uint8_t buf_[256];
    size_t n_ = 0;
    bool ok_ = true;
};

uint32_t morton(P16 p) {
    uint32_t r = 0;
    for (int b = 0; b < 16; ++b)
        r |= ((uint32_t(uint16_t(p.x)) >> b & 1) << (2 * b)) | ((uint32_t(uint16_t(p.y)) >> b & 1) << (2 * b + 1));
    return r;
}

struct Edge {  // geometry is stored in edge order: n interior points each
    uint16_t a, b, len, n;
    uint8_t cls, flags;
};

int buildGraph(RoadLines &lines, uint16_t extent, ByteSink &out) {
    Builder bd;
    bd.e = extent;
    bd.cell = std::max(16, extent / 32);
    if (lines.xy.size() / 2 > kMaxPoints) return -1;
    bd.clip(lines);
    if (bd.pts.size() > kMaxPoints || bd.pieces.size() > 65535) return -1;
    std::vector<Split> splits;
    bd.snapEnds(splits);
    bd.findCrossings(splits);
    bd.applySplits(splits);
    if (bd.pts.size() > kMaxPoints) return -1;

    // Nodes: line ends, split points and points shared by several lines.
    std::vector<uint16_t> order(bd.pts.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = uint16_t(i);
    std::sort(order.begin(), order.end(),
              [&](uint16_t a, uint16_t b) { return key(bd.pts[a]) < key(bd.pts[b]); });
    constexpr uint16_t kNone = 0xFFFF;
    std::vector<uint16_t> nodeOf(bd.pts.size(), kNone);
    std::vector<P16> nodes;
    for (size_t i = 0; i < order.size();) {
        size_t j = i;
        bool node = false;
        while (j < order.size() && bd.pts[order[j]] == bd.pts[order[i]]) node |= bd.isNode[order[j++]];
        if (node || j - i > 1) {
            if (nodes.size() >= 65534) return -1;
            for (size_t k = i; k < j; ++k) nodeOf[order[k]] = uint16_t(nodes.size());
            nodes.push_back(bd.pts[order[i]]);
        }
        i = j;
    }
    std::vector<uint16_t>().swap(order);
    std::vector<uint8_t>().swap(bd.isNode);

    std::vector<uint16_t>().swap(bd.pieceOf);
    std::vector<Edge> edges;
    std::vector<P16> geom;
    edges.reserve(bd.pts.size() - bd.pieces.size());  // each edge uses at least one segment
    for (const auto &pc : bd.pieces) {
        uint32_t i0 = pc.start;
        double len = 0;
        for (uint32_t k = 1; k < pc.count; ++k) {
            const uint32_t i = pc.start + k;
            const P16 a = bd.pts[i - 1], b = bd.pts[i];
            len += sqrt(double(b.x - a.x) * (b.x - a.x) + double(b.y - a.y) * (b.y - a.y));
            if (nodeOf[i] == kNone && len < kMaxEdgeLen) continue;
            if (nodeOf[i] == kNone) {
                if (nodes.size() >= 65534) return -1;
                nodeOf[i] = uint16_t(nodes.size());
                nodes.push_back(b);
            }
            const uint16_t na = nodeOf[i0], nb = nodeOf[i];
            if (na != nb) {
                if (edges.size() >= 65535) return -1;
                Edge ed;
                ed.a = na;
                ed.b = nb;
                ed.len = uint16_t(std::max(1L, lround(len)));
                ed.n = uint16_t(i - i0 - 1);
                ed.cls = pc.cls;
                ed.flags = pc.flags;
                for (uint32_t g = i0 + 1; g < i; ++g) geom.push_back(bd.pts[g]);
                edges.push_back(ed);
            }
            i0 = i;
            len = 0;
        }
    }
    std::vector<P16>().swap(bd.pts);
    std::vector<uint16_t>().swap(nodeOf);
    std::vector<Piece>().swap(bd.pieces);
    edges.shrink_to_fit();

    // Spatial node order keeps neighbours close together in the file (fewer SD reads).
    const size_t nn = nodes.size();
    std::vector<uint16_t> perm(nn), rank(nn);
    for (size_t i = 0; i < nn; ++i) perm[i] = uint16_t(i);
    std::sort(perm.begin(), perm.end(), [&](uint16_t a, uint16_t b) { return morton(nodes[a]) < morton(nodes[b]); });
    for (size_t i = 0; i < nn; ++i) rank[perm[i]] = uint16_t(i);
    std::vector<P16> sorted(nn);
    for (size_t i = 0; i < nn; ++i) sorted[rank[i]] = nodes[i];
    nodes.swap(sorted);
    std::vector<P16>().swap(sorted);
    for (auto &ed : edges) {
        ed.a = rank[ed.a];
        ed.b = rank[ed.b];
    }
    std::vector<uint16_t>().swap(perm);
    std::vector<uint16_t>().swap(rank);

    std::vector<uint32_t> first(nn + 1, 0);
    for (const auto &ed : edges) {
        ++first[ed.a + 1];
        ++first[ed.b + 1];
    }
    for (size_t i = 1; i <= nn; ++i) first[i] += first[i - 1];
    // adjacency as (edge index << 1 | reversed)
    std::vector<uint32_t> adj(first[nn]);
    {
        std::vector<uint32_t> fill(first.begin(), first.end() - 1);
        for (size_t k = 0; k < edges.size(); ++k) {
            adj[fill[edges[k].a]++] = uint32_t(k) << 1;
            adj[fill[edges[k].b]++] = uint32_t(k) << 1 | 1;
        }
    }

    std::vector<std::pair<int16_t, uint16_t>> bounds[4];
    for (size_t i = 0; i < nn; ++i) {
        const P16 p = nodes[i];
        if (p.x == 0) bounds[0].push_back({p.y, uint16_t(i)});
        if (p.x == extent) bounds[1].push_back({p.y, uint16_t(i)});
        if (p.y == 0) bounds[2].push_back({p.x, uint16_t(i)});
        if (p.y == extent) bounds[3].push_back({p.x, uint16_t(i)});
    }
    for (auto &b : bounds) std::sort(b.begin(), b.end());

    BufWriter w(out);
    w.raw(kMagic, 4);
    w.u16(extent);
    w.u16(uint16_t(nn));
    w.u16(uint16_t(edges.size()));
    for (auto &b : bounds) w.u16(uint16_t(b.size()));
    w.u16(0);
    w.u32(uint32_t(adj.size()));
    w.u32(uint32_t(geom.size()));
    w.u32(0);
    for (size_t i = 0; i <= nn; ++i) {
        w.u16(uint16_t(i < nn ? nodes[i].x : 0));
        w.u16(uint16_t(i < nn ? nodes[i].y : 0));
        w.u32(first[i]);
    }
    for (size_t i = 0; i < nn; ++i) {
        for (uint32_t k = first[i]; k < first[i + 1]; ++k) {
            const Edge &ed = edges[adj[k] >> 1];
            const bool rev = adj[k] & 1;
            const uint16_t to = rev ? ed.a : ed.b;
            const uint8_t noDrive = rev ? (ed.flags & kRoadOneway) : (ed.flags & kRoadOnewayRev);
            w.u16(to);
            w.u16(uint16_t(nodes[to].x));
            w.u16(uint16_t(nodes[to].y));
            w.u16(ed.len);
            w.u16(uint16_t(adj[k] >> 1));
            w.u8(ed.cls);
            w.u8(uint8_t((noDrive ? kAdjNoDrive : 0) | (rev ? kAdjReversed : 0)));
        }
    }
    uint32_t g0 = 0;
    for (const auto &ed : edges) {
        const uint32_t gStart = g0;
        g0 += ed.n;
        w.u16(ed.a);
        w.u16(ed.b);
        w.u16(ed.len);
        w.u16(ed.n);
        w.u32(gStart);
        w.u8(ed.cls);
        w.u8(ed.flags & (kRoadOneway | kRoadOnewayRev));
        P16 lo = nodes[ed.a], hi = lo;
        auto grow = [&](P16 p) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        };
        grow(nodes[ed.b]);
        for (uint32_t i = 0; i < ed.n; ++i) grow(geom[gStart + i]);
        w.u16(uint16_t(lo.x));
        w.u16(uint16_t(lo.y));
        w.u16(uint16_t(hi.x));
        w.u16(uint16_t(hi.y));
    }
    for (const auto &g : geom) {
        w.u16(uint16_t(g.x));
        w.u16(uint16_t(g.y));
    }
    for (auto &b : bounds)
        for (auto &e : b) {
            w.u16(uint16_t(e.first));
            w.u16(e.second);
        }
    if (!w.flush()) return -1;
    return int(nn);
}

}  // namespace

bool buildRoadPath(const char *root, int z, int x, int y, char *out, size_t n) {
    return fits(snprintf(out, n, "%s/%d/%d/%d.bin", root, z, x, y), n);
}

bool buildRoadDir(const char *root, int z, int x, char *out, size_t n) {
    return fits(snprintf(out, n, "%s/%d/%d", root, z, x), n);
}

bool parseMvtRoads(ByteSource &mvt, RoadLines &out, uint16_t &extent) {
    extent = 4096;
    Cursor top(mvt, 0, mvt.size());
    uint32_t num, wire;
    while (!top.done()) {
        if (!top.field(num, wire)) return false;
        if (num == 3 && wire == 2) {
            size_t s, l;
            if (!top.lenDelim(s, l) || parseLayer(mvt, s, l, out, extent) < 0) return false;
        } else if (!top.skipValue(wire)) {
            return false;
        }
    }
    return true;
}

int buildRoadGraph(RoadLines &lines, uint16_t extent, ByteSink &out) {
    if (extent == 0 || extent > 16384) return -1;
    try {
        return buildGraph(lines, extent, out);
    } catch (const std::bad_alloc &) {
        return -1;
    }
}

int convertMvtRoads(ByteSource &mvt, ByteSink &out) {
    try {
        RoadLines lines;
        uint16_t extent = 4096;
        if (!parseMvtRoads(mvt, lines, extent)) return -1;
        return buildGraph(lines, extent, out);
    } catch (const std::bad_alloc &) {
        return -1;
    }
}

// ---------------------------------------------------------------- routing

namespace {

constexpr uint32_t kSpecialSlot = 127;
constexpr uint32_t kStartKey = kSpecialSlot << 16, kEndKey = kSpecialSlot << 16 | 1;
constexpr uint32_t kEmpty = 0xFFFFFFFF, kNoParent = 0x7FFFFFFF, kClosed = 0x80000000;
constexpr int kMaxSlots = 126;

const float kDriveSpeed[kRoadClassCount] = {29, 24, 18, 15, 13, 10, 5, 0, 0};  // m/s
constexpr float kWalkSpeed = 1.4f;

uint16_t rd16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t *p) { return uint32_t(rd16(p)) | (uint32_t(rd16(p + 2)) << 16); }

struct Tile {
    int tx = 0, ty = 0;
    bool valid = false;
    uint16_t extent = 4096, nNodes = 0, nEdges = 0, nb[4] = {0, 0, 0, 0};
    uint32_t nAdj = 0, nGeom = 0;
    uint32_t adjOff = 0, edgeOff = 0, geomOff = 0, bOff[4] = {0, 0, 0, 0};
    double mPerUnit = 1;
};

struct Snap {
    int slot = -1;
    uint16_t a = 0, b = 0;
    uint8_t cls = 0, flags = 0;
    std::vector<P16> poly;  // a .. b
    int seg = 0;
    double sx = 0, sy = 0;  // snapped point (tile units)
    double offA = 0, len = 0;
};

class Router {
public:
    Router(RoadTiles &tiles, const RouteOptions &opt) : tiles_(tiles), opt_(opt) {}

    RouteStatus run(LatLon from, LatLon to, RouteResult &out);

private:
    RoadTiles &tiles_;
    const RouteOptions &opt_;
    std::vector<Tile> slots_;
    Snap start_, end_;
    double goalX_ = 0, goalY_ = 0;  // goal in z14 tile coordinates
    double mPerTile_ = 1;
    float vmax_ = 1;

    std::vector<uint32_t> keys_, parent_;
    std::vector<float> g_;
    uint32_t mask_ = 0, used_ = 0;
    struct HeapItem {
        float f;
        uint32_t key;
        bool operator<(const HeapItem &o) const { return f > o.f; }
    };
    std::vector<HeapItem> heap_;
    size_t heapCap_ = 0;
    bool overflow_ = false;

    bool allowed(uint8_t cls) const {
        if (cls >= kRoadClassCount) return false;
        return opt_.mode == RouteMode::Walk ? cls != kRoadMotorway : kDriveSpeed[cls] > 0;
    }
    float cost(int slot, double units, uint8_t cls) const {
        const double m = units * slots_[slot].mPerUnit;
        return float(m / (opt_.mode == RouteMode::Walk ? kWalkSpeed : kDriveSpeed[cls]));
    }
    float heur(int slot, double x, double y) const {
        const Tile &t = slots_[slot];
        const double dx = t.tx + x / t.extent - goalX_, dy = t.ty + y / t.extent - goalY_;
        return float(sqrt(dx * dx + dy * dy) * mPerTile_ / vmax_ * opt_.heuristicWeight);
    }

    int slotFor(int tx, int ty);
    bool read(int slot, uint32_t off, uint8_t *buf, size_t n) {
        ByteSource *s = tiles_.tile(slots_[slot].tx, slots_[slot].ty);
        return s && s->read(off, buf, n);
    }
    bool readNode(int slot, uint16_t n, P16 &p, uint32_t &a0, uint32_t &a1) {
        uint8_t b[16];
        if (!read(slot, kHeaderSize + uint32_t(n) * kNodeSize, b, 16)) return false;
        p = {int16_t(rd16(b)), int16_t(rd16(b + 2))};
        a0 = rd32(b + 4);
        a1 = rd32(b + 12);
        return a0 <= a1 && a1 <= slots_[slot].nAdj;
    }
    bool readEdgePoly(int slot, uint16_t e, std::vector<P16> &poly, uint16_t *pa = nullptr, uint16_t *pb = nullptr);
    bool snap(LatLon p, Snap &s, bool &anyData);

    int find(uint32_t key, bool insert);
    void relax(uint32_t from, float gFrom, uint32_t to, float add, int slot, double x, double y);
    bool expand(uint32_t key, float g);
};

int Router::slotFor(int tx, int ty) {
    for (size_t i = 0; i < slots_.size(); ++i)
        if (slots_[i].tx == tx && slots_[i].ty == ty) return slots_[i].valid ? int(i) : -1;
    if (slots_.size() >= kMaxSlots) return -1;
    Tile t;
    t.tx = tx;
    t.ty = ty;
    const int n = 1 << kStreetZoom;
    ByteSource *s = (tx >= 0 && ty >= 0 && tx < n && ty < n) ? tiles_.tile(tx, ty) : nullptr;
    uint8_t h[kHeaderSize];
    if (s && s->size() >= kHeaderSize && s->read(0, h, kHeaderSize) && !memcmp(h, kMagic, 4)) {
        t.extent = rd16(h + 4);
        t.nNodes = rd16(h + 6);
        t.nEdges = rd16(h + 8);
        for (int i = 0; i < 4; ++i) t.nb[i] = rd16(h + 10 + 2 * i);
        t.nAdj = rd32(h + 20);
        t.nGeom = rd32(h + 24);
        t.adjOff = uint32_t(kHeaderSize + (t.nNodes + 1u) * kNodeSize);
        t.edgeOff = t.adjOff + t.nAdj * kAdjSize;
        t.geomOff = t.edgeOff + t.nEdges * kEdgeSize;
        t.bOff[0] = t.geomOff + t.nGeom * kGeomSize;
        for (int i = 1; i < 4; ++i) t.bOff[i] = t.bOff[i - 1] + t.nb[i - 1] * kBoundSize;
        const double latC = worldPxToLatLon((tx + 0.5) * kTileSize, (ty + 0.5) * kTileSize, kStreetZoom).lat;
        t.mPerUnit = 2 * M_PI * kEarthRadiusM * cos(latC * M_PI / 180) / double(n) / t.extent;
        t.valid = t.extent > 0 && s->size() >= t.bOff[3] + t.nb[3] * kBoundSize;
    }
    slots_.push_back(t);
    return t.valid ? int(slots_.size() - 1) : -1;
}

bool Router::readEdgePoly(int slot, uint16_t e, std::vector<P16> &poly, uint16_t *pa, uint16_t *pb) {
    const Tile &t = slots_[slot];
    uint8_t b[kEdgeSize];
    if (e >= t.nEdges || !read(slot, t.edgeOff + uint32_t(e) * kEdgeSize, b, kEdgeSize)) return false;
    const uint16_t na = rd16(b), nb = rd16(b + 2), ng = rd16(b + 6);
    const uint32_t g0 = rd32(b + 8);
    if (na >= t.nNodes || nb >= t.nNodes || uint64_t(g0) + ng > t.nGeom) return false;
    uint32_t x0, x1;
    P16 pA, pB;
    if (!readNode(slot, na, pA, x0, x1) || !readNode(slot, nb, pB, x0, x1)) return false;
    poly.clear();
    poly.push_back(pA);
    uint8_t buf[64];
    for (uint32_t i = 0; i < ng;) {
        const uint32_t k = std::min<uint32_t>(ng - i, sizeof buf / kGeomSize);
        if (!read(slot, t.geomOff + (g0 + i) * kGeomSize, buf, k * kGeomSize)) return false;
        for (uint32_t j = 0; j < k; ++j) poly.push_back({int16_t(rd16(buf + 4 * j)), int16_t(rd16(buf + 4 * j + 2))});
        i += k;
    }
    poly.push_back(pB);
    if (pa) *pa = na;
    if (pb) *pb = nb;
    return true;
}

bool Router::snap(LatLon p, Snap &s, bool &anyData) {
    const WorldPx w = latLonToWorldPx(p.lat, p.lon, kStreetZoom);
    const int tx = int(floor(w.x / kTileSize)), ty = int(floor(w.y / kTileSize));
    const double fx = w.x / kTileSize - tx, fy = w.y / kTileSize - ty;
    double bestM = opt_.maxSnapM;
    bool found = false;
    std::vector<P16> poly;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int slot = slotFor(tx + dx, ty + dy);
            if (slot < 0) continue;
            const Tile &t = slots_[slot];
            if (!dx && !dy) anyData = true;
            // Skip neighbours farther than the best match so far.
            const double ex = dx < 0 ? fx : dx > 0 ? 1 - fx : 0, ey = dy < 0 ? fy : dy > 0 ? 1 - fy : 0;
            if (sqrt(ex * ex + ey * ey) * t.extent * t.mPerUnit > bestM) continue;
            const double px = (fx - dx) * t.extent, py = (fy - dy) * t.extent;
            uint8_t eb[kEdgeSize * 16];
            for (uint32_t e0 = 0; e0 < t.nEdges; e0 += 16) {
                const uint32_t k = std::min<uint32_t>(16, t.nEdges - e0);
                if (!read(slot, t.edgeOff + e0 * kEdgeSize, eb, k * kEdgeSize)) return false;
                for (uint32_t j = 0; j < k; ++j) {
                    const uint16_t e = uint16_t(e0 + j);
                    const uint8_t *er = eb + j * kEdgeSize;
                    const uint8_t cls = er[12], flags = er[13];
                    if (!allowed(cls)) continue;
                    const double bx = std::max(0.0, std::max(int16_t(rd16(er + 14)) - px, px - int16_t(rd16(er + 18))));
                    const double by = std::max(0.0, std::max(int16_t(rd16(er + 16)) - py, py - int16_t(rd16(er + 20))));
                    if (sqrt(bx * bx + by * by) * t.mPerUnit >= bestM) continue;
                    uint16_t ea, ebn;
                    if (!readEdgePoly(slot, e, poly, &ea, &ebn)) return false;
                    double acc = 0;
                    for (size_t i = 0; i + 1 < poly.size(); ++i) {
                        double d2;
                        const double tt = projT(px, py, poly[i], poly[i + 1], d2);
                        const double segLen = hypot(poly[i + 1].x - poly[i].x, poly[i + 1].y - poly[i].y);
                        const double dm = sqrt(d2) * t.mPerUnit;
                        if (dm < bestM) {
                            bestM = dm;
                            found = true;
                            s.slot = slot;
                            s.a = ea;
                            s.b = ebn;
                            s.cls = cls;
                            s.flags = flags;
                            s.poly = poly;
                            s.seg = int(i);
                            s.sx = poly[i].x + tt * (poly[i + 1].x - poly[i].x);
                            s.sy = poly[i].y + tt * (poly[i + 1].y - poly[i].y);
                            s.offA = acc + tt * segLen;
                        }
                        acc += segLen;
                    }
                }
            }
        }
    }
    if (found) {
        s.len = 0;
        for (size_t i = 0; i + 1 < s.poly.size(); ++i)
            s.len += hypot(s.poly[i + 1].x - s.poly[i].x, s.poly[i + 1].y - s.poly[i].y);
    }
    return found;
}

int Router::find(uint32_t key, bool insert) {
    uint32_t h = (key * 2654435761u) & mask_;
    while (keys_[h] != kEmpty) {
        if (keys_[h] == key) return int(h);
        h = (h + 1) & mask_;
    }
    if (!insert) return -1;
    if (used_ >= opt_.maxNodes) {
        overflow_ = true;
        return -1;
    }
    ++used_;
    keys_[h] = key;
    g_[h] = INFINITY;
    parent_[h] = kNoParent;
    return int(h);
}

void Router::relax(uint32_t from, float gFrom, uint32_t to, float add, int slot, double x, double y) {
    const int i = find(to, true);
    if (i < 0 || (parent_[i] & kClosed)) return;
    const float ng = gFrom + add;
    if (!(ng < g_[i])) return;
    g_[i] = ng;
    parent_[i] = from;
    if (heap_.size() >= heapCap_) {
        overflow_ = true;
        return;
    }
    heap_.push_back({ng + (to == kEndKey ? 0.f : heur(slot, x, y)), to});
    std::push_heap(heap_.begin(), heap_.end());
}

bool Router::expand(uint32_t key, float g) {
    const bool walk = opt_.mode == RouteMode::Walk;
    if (key == kStartKey) {
        const Snap &s = start_;
        const P16 pa = s.poly.front(), pb = s.poly.back();
        if (walk || !(s.flags & kRoadOneway))
            relax(key, g, uint32_t(s.slot) << 16 | s.a, cost(s.slot, s.offA, s.cls), s.slot, pa.x, pa.y);
        if (walk || !(s.flags & kRoadOnewayRev))
            relax(key, g, uint32_t(s.slot) << 16 | s.b, cost(s.slot, s.len - s.offA, s.cls), s.slot, pb.x, pb.y);
        if (end_.slot == s.slot && end_.a == s.a && end_.b == s.b) {
            const double d = end_.offA - s.offA;
            if (d >= 0 ? (walk || !(s.flags & kRoadOnewayRev)) : (walk || !(s.flags & kRoadOneway)))
                relax(key, g, kEndKey, cost(s.slot, fabs(d), s.cls), 0, 0, 0);
        }
        return true;
    }
    const int slot = int(key >> 16);
    const uint16_t n = uint16_t(key & 0xFFFF);
    const Tile &t = slots_[slot];
    P16 p;
    uint32_t a0, a1;
    if (!readNode(slot, n, p, a0, a1)) return false;
    uint8_t buf[kAdjSize * 8];
    for (uint32_t ai = a0; ai < a1;) {
        const uint32_t k = std::min<uint32_t>(8, a1 - ai);
        if (!read(slot, t.adjOff + ai * kAdjSize, buf, k * kAdjSize)) return false;
        for (uint32_t j = 0; j < k; ++j) {
            const uint8_t *e = buf + j * kAdjSize;
            const uint8_t cls = e[10], fl = e[11];
            if (!allowed(cls) || (!walk && (fl & kAdjNoDrive))) continue;
            const uint16_t to = rd16(e);
            if (to >= t.nNodes) return false;
            relax(key, g, uint32_t(slot) << 16 | to, cost(slot, rd16(e + 6), cls), slot, int16_t(rd16(e + 2)),
                  int16_t(rd16(e + 4)));
        }
        ai += k;
    }
    if (end_.slot == slot) {
        if (n == end_.a && (walk || !(end_.flags & kRoadOnewayRev)))
            relax(key, g, kEndKey, cost(slot, end_.offA, end_.cls), 0, 0, 0);
        if (n == end_.b && (walk || !(end_.flags & kRoadOneway)))
            relax(key, g, kEndKey, cost(slot, end_.len - end_.offA, end_.cls), 0, 0, 0);
    }
    // Lines are cut at tile borders; continue on the neighbour tile's matching border node.
    const int ext = t.extent;
    const double tol = kBorderUnits * ext / 4096.0;
    struct Side {
        bool on;
        int dx, dy, side, pos;
    } sides[4] = {{p.x == 0, -1, 0, 1, p.y},
                  {p.x == ext, 1, 0, 0, p.y},
                  {p.y == 0, 0, -1, 3, p.x},
                  {p.y == ext, 0, 1, 2, p.x}};
    const int tx = t.tx, ty = t.ty;
    for (const auto &sd : sides) {
        if (!sd.on) continue;
        const int ns = slotFor(tx + sd.dx, ty + sd.dy);
        if (ns < 0) continue;
        const Tile &nt = slots_[ns];
        const double pos = double(sd.pos) * nt.extent / ext;
        // binary search for the closest border node
        uint32_t lo = 0, hi = nt.nb[sd.side];
        while (lo < hi) {
            const uint32_t mid = (lo + hi) / 2;
            uint8_t b[4];
            if (!read(ns, nt.bOff[sd.side] + mid * kBoundSize, b, 4)) return false;
            if (int16_t(rd16(b)) < pos) lo = mid + 1;
            else hi = mid;
        }
        int bestNode = -1;
        double bestD = tol;
        for (uint32_t c = lo > 0 ? lo - 1 : 0; c <= lo && c < nt.nb[sd.side]; ++c) {
            uint8_t b[4];
            if (!read(ns, nt.bOff[sd.side] + c * kBoundSize, b, 4)) return false;
            const double d = fabs(int16_t(rd16(b)) - pos);
            if (d <= bestD) {
                bestD = d;
                bestNode = rd16(b + 2);
            }
        }
        if (bestNode < 0 || bestNode >= nt.nNodes) continue;
        const double nx = sd.dx < 0 ? nt.extent : sd.dx > 0 ? 0 : pos;
        const double ny = sd.dy < 0 ? nt.extent : sd.dy > 0 ? 0 : pos;
        relax(key, g, uint32_t(ns) << 16 | uint16_t(bestNode), 0, ns, nx, ny);
    }
    return true;
}

RouteStatus Router::run(LatLon from, LatLon to, RouteResult &out) {
    out = RouteResult();
    bool data = false;
    if (!snap(from, start_, data)) return data ? RouteStatus::NoRoadNearStart : RouteStatus::NoData;
    data = false;
    if (!snap(to, end_, data)) return data ? RouteStatus::NoRoadNearEnd : RouteStatus::NoData;

    const WorldPx gw = latLonToWorldPx(to.lat, to.lon, kStreetZoom);
    goalX_ = gw.x / kTileSize;
    goalY_ = gw.y / kTileSize;
    mPerTile_ = 2 * M_PI * kEarthRadiusM * cos(to.lat * M_PI / 180) / double(1 << kStreetZoom);
    vmax_ = opt_.mode == RouteMode::Walk ? kWalkSpeed : kDriveSpeed[kRoadMotorway];

    uint32_t cap = 1024;
    while (cap < opt_.maxNodes + opt_.maxNodes / 3) cap <<= 1;
    keys_.assign(cap, kEmpty);
    g_.assign(cap, INFINITY);
    parent_.assign(cap, kNoParent);
    mask_ = cap - 1;
    heapCap_ = size_t(opt_.maxNodes) * 2 + 64;
    heap_.reserve(heapCap_);

    const int s0 = find(kStartKey, true);
    g_[s0] = 0;
    heap_.push_back({0, kStartKey});
    bool reached = false;
    while (!heap_.empty() && !overflow_) {
        std::pop_heap(heap_.begin(), heap_.end());
        const HeapItem it = heap_.back();
        heap_.pop_back();
        const int i = find(it.key, false);
        if (i < 0 || (parent_[i] & kClosed)) continue;
        parent_[i] |= kClosed;
        if (it.key == kEndKey) {
            reached = true;
            break;
        }
        if (!expand(it.key, g_[i])) return RouteStatus::Error;
        if ((++out.visited & 255) == 0 && opt_.poll && opt_.poll(opt_.ctx, out.visited)) return RouteStatus::Cancelled;
    }
    if (!reached) return overflow_ ? RouteStatus::TooFar : RouteStatus::NoRoute;

    // Walk back from the end.
    std::vector<uint32_t> chain;
    for (uint32_t k = kEndKey; k != kNoParent;) {
        chain.push_back(k);
        const int i = find(k, false);
        if (i < 0 || chain.size() > used_ + 1) return RouteStatus::Error;
        k = parent_[i] & ~kClosed;
    }
    std::reverse(chain.begin(), chain.end());
    out.timeS = g_[find(kEndKey, false)];
    std::vector<uint32_t>().swap(keys_);
    std::vector<uint32_t>().swap(parent_);
    std::vector<float>().swap(g_);
    std::vector<HeapItem>().swap(heap_);

    auto emit = [&](int slot, double x, double y) {
        const Tile &t = slots_[slot];
        const LatLon ll = worldPxToLatLon((t.tx + x / t.extent) * kTileSize, (t.ty + y / t.extent) * kTileSize,
                                          kStreetZoom);
        if (!out.pts.empty() && fabs(out.pts.back().lat - ll.lat) < 1e-8 && fabs(out.pts.back().lon - ll.lon) < 1e-8)
            return;
        out.pts.push_back(ll);
    };
    auto emitPoly = [&](int slot, const std::vector<P16> &poly, int from, int to) {  // inclusive, either direction
        if (from <= to)
            for (int i = from; i <= to; ++i) emit(slot, poly[i].x, poly[i].y);
        else
            for (int i = from; i >= to; --i) emit(slot, poly[i].x, poly[i].y);
    };
    const int last = int(start_.poly.size()) - 1;
    std::vector<P16> poly;
    for (size_t c = 0; c + 1 < chain.size(); ++c) {
        const uint32_t u = chain[c], v = chain[c + 1];
        if (u == kStartKey) {
            emit(start_.slot, start_.sx, start_.sy);
            if (v == kEndKey) {  // both on the same edge
                if (end_.offA >= start_.offA)
                    for (int i = start_.seg + 1; i <= end_.seg; ++i) emit(start_.slot, start_.poly[i].x, start_.poly[i].y);
                else
                    for (int i = start_.seg; i > end_.seg; --i) emit(start_.slot, start_.poly[i].x, start_.poly[i].y);
            } else if ((v & 0xFFFF) == start_.a) {
                emitPoly(start_.slot, start_.poly, start_.seg, 0);
            } else {
                emitPoly(start_.slot, start_.poly, start_.seg + 1, last);
            }
            if (v != kEndKey) continue;
        }
        if (v == kEndKey) {
            if (u != kStartKey) {
                const int el = int(end_.poly.size()) - 1;
                if ((u & 0xFFFF) == end_.a) emitPoly(end_.slot, end_.poly, 0, end_.seg);
                else emitPoly(end_.slot, end_.poly, el, end_.seg + 1);
            }
            emit(end_.slot, end_.sx, end_.sy);
            continue;
        }
        const int su = int(u >> 16), sv = int(v >> 16);
        const uint16_t nu = uint16_t(u & 0xFFFF), nv = uint16_t(v & 0xFFFF);
        if (su != sv) {
            P16 p;
            uint32_t a0, a1;
            if (!readNode(sv, nv, p, a0, a1)) return RouteStatus::Error;
            emit(sv, p.x, p.y);
            continue;
        }
        // Cheapest usable edge u -> v.
        P16 p;
        uint32_t a0, a1;
        if (!readNode(su, nu, p, a0, a1)) return RouteStatus::Error;
        int bestEdge = -1;
        bool bestRev = false;
        float bestCost = INFINITY;
        for (uint32_t ai = a0; ai < a1; ++ai) {
            uint8_t e[kAdjSize];
            if (!read(su, slots_[su].adjOff + ai * kAdjSize, e, kAdjSize)) return RouteStatus::Error;
            if (rd16(e) != nv || !allowed(e[10]) || (opt_.mode == RouteMode::Drive && (e[11] & kAdjNoDrive))) continue;
            const float cst = cost(su, rd16(e + 6), e[10]);
            if (cst < bestCost) {
                bestCost = cst;
                bestEdge = rd16(e + 8);
                bestRev = e[11] & kAdjReversed;
            }
        }
        if (bestEdge < 0 || !readEdgePoly(su, uint16_t(bestEdge), poly)) return RouteStatus::Error;
        const int pl = int(poly.size()) - 1;
        if (bestRev) emitPoly(su, poly, pl, 0);
        else emitPoly(su, poly, 0, pl);
    }
    for (size_t i = 1; i < out.pts.size(); ++i) out.lengthM += haversineM(out.pts[i - 1], out.pts[i]);
    return out.pts.size() >= 2 ? RouteStatus::Ok : RouteStatus::NoRoute;
}

}  // namespace

RouteStatus findRoute(RoadTiles &tiles, LatLon from, LatLon to, const RouteOptions &opt, RouteResult &out) {
    try {
        Router r(tiles, opt);
        return r.run(from, to, out);
    } catch (const std::bad_alloc &) {
        out = RouteResult();
        return RouteStatus::TooFar;
    }
}

const char *routeStatusText(RouteStatus s) {
    switch (s) {
        case RouteStatus::Ok: return "Route found";
        case RouteStatus::NoData: return "No road data here. Download this area with a MapTiler key.";
        case RouteStatus::NoRoadNearStart: return "No road near the start point.";
        case RouteStatus::NoRoadNearEnd: return "No road near the end point.";
        case RouteStatus::NoRoute: return "No route between these points in the downloaded roads.";
        case RouteStatus::TooFar: return "Route too long for the device's memory. Set closer points.";
        case RouteStatus::Cancelled: return "Routing cancelled.";
        default: return "Road data could not be read.";
    }
}

}  // namespace nav
