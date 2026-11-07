#include "streets.h"

#include "tilesource.h"

#include <math.h>
#include <stdio.h>
#include <vector>

namespace nav {

namespace {

constexpr char kMagic[4] = {'G', 'N', 'S', '1'};
constexpr size_t kHeaderSize = 6;
constexpr size_t kMaxPartPoints = 2000;
constexpr size_t kMaxName = 63;

bool fits(int w, size_t n) { return w > 0 && size_t(w) < n; }

// Sequential protobuf reader over a ByteSource range, with a small read-ahead buffer.
class Cursor {
public:
    Cursor(ByteSource &s, size_t pos, size_t end) : s_(s), pos_(pos), end_(end) {}
    bool done() const { return pos_ >= end_; }

    bool byte(uint8_t &b) {
        if (pos_ >= end_) return false;
        if (pos_ < bufStart_ || pos_ >= bufStart_ + bufLen_) {
            size_t n = end_ - pos_;
            if (n > sizeof buf_) n = sizeof buf_;
            if (!s_.read(pos_, buf_, n)) return false;
            bufStart_ = pos_;
            bufLen_ = n;
        }
        b = buf_[pos_ - bufStart_];
        ++pos_;
        return true;
    }
    bool varint(uint64_t &v) {
        v = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            uint8_t b;
            if (!byte(b)) return false;
            v |= uint64_t(b & 0x7f) << shift;
            if (!(b & 0x80)) return true;
        }
        return false;
    }
    bool skip(size_t n) {
        if (n > end_ - pos_) return false;
        pos_ += n;
        return true;
    }
    bool field(uint32_t &num, uint32_t &wire) {
        uint64_t k;
        if (!varint(k)) return false;
        num = uint32_t(k >> 3);
        wire = uint32_t(k & 7);
        return true;
    }
    bool lenDelim(size_t &start, size_t &len) {
        uint64_t l;
        if (!varint(l) || l > end_ - pos_) return false;
        start = pos_;
        len = size_t(l);
        pos_ += len;
        return true;
    }
    bool skipValue(uint32_t wire) {
        uint64_t v;
        size_t s, l;
        switch (wire) {
            case 0: return varint(v);
            case 1: return skip(8);
            case 2: return lenDelim(s, l);
            case 5: return skip(4);
            default: return false;
        }
    }

private:
    ByteSource &s_;
    size_t pos_, end_;
    uint8_t buf_[96];
    size_t bufStart_ = 0, bufLen_ = 0;
};

struct Span {
    uint32_t start = 0;
    uint32_t len = 0;
    bool valid = false;
};

bool readSpan(ByteSource &s, const Span &sp, char *out, size_t n) {
    size_t len = sp.len < n - 1 ? sp.len : n - 1;
    if (!s.read(sp.start, reinterpret_cast<uint8_t *>(out), len)) return false;
    out[len] = 0;
    return true;
}

bool spanEquals(ByteSource &s, size_t start, size_t len, const char *str) {
    char buf[32];
    if (len != strlen(str) || len >= sizeof buf) return false;
    if (!s.read(start, reinterpret_cast<uint8_t *>(buf), len)) return false;
    return memcmp(buf, str, len) == 0;
}

int16_t clamp16(int64_t v) { return int16_t(v < -32768 ? -32768 : v > 32767 ? 32767 : v); }

bool put16(ByteSink &out, uint16_t v) {
    const uint8_t b[2] = {uint8_t(v & 0xff), uint8_t(v >> 8)};
    return out.write(b, 2);
}

class StreetWriter {
public:
    explicit StreetWriter(ByteSink &out) : out_(out) {}
    bool header(uint16_t extent) {
        if (headerDone_) return true;
        headerDone_ = true;
        return out_.write(reinterpret_cast<const uint8_t *>(kMagic), 4) && put16(out_, extent);
    }
    bool part(const char *name, const std::vector<int16_t> &pts) {
        const size_t np = pts.size() / 2;
        if (np < 2) return true;
        const uint8_t nl = uint8_t(strlen(name));
        bool ok = out_.write(&nl, 1) && out_.write(reinterpret_cast<const uint8_t *>(name), nl) &&
                  put16(out_, uint16_t(np));
        for (size_t i = 0; ok && i < pts.size(); ++i) ok = put16(out_, uint16_t(pts[i]));
        if (ok) ++parts_;
        return ok;
    }
    bool headerDone() const { return headerDone_; }
    int parts() const { return parts_; }

private:
    ByteSink &out_;
    bool headerDone_ = false;
    int parts_ = 0;
};

bool writeGeometry(ByteSource &src, size_t start, size_t len, const char *name, StreetWriter &w) {
    Cursor g(src, start, start + len);
    std::vector<int16_t> pts;
    int64_t x = 0, y = 0;
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
            if (id == 1 || pts.size() / 2 >= kMaxPartPoints) {
                const bool cont = id == 2;  // split an overlong line, keeping it connected
                int16_t lx = 0, ly = 0;
                if (cont) {
                    lx = pts[pts.size() - 2];
                    ly = pts.back();
                }
                if (!w.part(name, pts)) return false;
                pts.clear();
                if (cont) {
                    pts.push_back(lx);
                    pts.push_back(ly);
                }
            }
            pts.push_back(clamp16(x));
            pts.push_back(clamp16(y));
        }
    }
    return w.part(name, pts);
}

int processLayer(ByteSource &src, size_t start, size_t len, StreetWriter &w) {
    // Pass 1: layer name, keys, string values, extent.
    bool isTarget = false;
    int nameKey = -1, latinKey = -1, keyIdx = 0;
    uint64_t extent = 4096;
    std::vector<Span> values;
    {
        Cursor c(src, start, start + len);
        uint32_t num, wire;
        while (!c.done()) {
            if (!c.field(num, wire)) return -1;
            size_t s, l;
            if (num == 1 && wire == 2) {
                if (!c.lenDelim(s, l)) return -1;
                isTarget = spanEquals(src, s, l, "transportation_name");
            } else if (num == 3 && wire == 2) {
                if (!c.lenDelim(s, l)) return -1;
                if (spanEquals(src, s, l, "name")) nameKey = keyIdx;
                else if (spanEquals(src, s, l, "name:latin")) latinKey = keyIdx;
                ++keyIdx;
            } else if (num == 4 && wire == 2) {
                if (!c.lenDelim(s, l)) return -1;
                Span v;
                Cursor vc(src, s, s + l);
                uint32_t vn, vw;
                while (!vc.done()) {
                    if (!vc.field(vn, vw)) return -1;
                    size_t ss, sl;
                    if (vn == 1 && vw == 2) {
                        if (!vc.lenDelim(ss, sl)) return -1;
                        v.start = uint32_t(ss);
                        v.len = uint32_t(sl);
                        v.valid = true;
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
    if (!isTarget) return 0;
    if (extent == 0 || extent > 32767) return -1;
    if (!w.header(uint16_t(extent))) return -1;
    if (nameKey < 0 && latinKey < 0) return 0;

    // Pass 2: named line features.
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
        int nameVal = -1, latinVal = -1;
        Cursor t(src, ts, ts + tl);
        while (!t.done()) {
            uint64_t k, v;
            if (!t.varint(k) || !t.varint(v)) return -1;
            if (int(k) == nameKey) nameVal = int(v);
            else if (int(k) == latinKey) latinVal = int(v);
        }
        auto usable = [&](int idx) { return idx >= 0 && size_t(idx) < values.size() && values[idx].valid; };
        const int pick = usable(latinVal) ? latinVal : usable(nameVal) ? nameVal : -1;
        if (pick < 0) continue;
        char raw[128], name[kMaxName + 1];
        if (!readSpan(src, values[pick], raw, sizeof raw)) return -1;
        asciiFold(raw, strlen(raw), name, sizeof name);
        if (!name[0]) continue;
        if (!writeGeometry(src, gs, gl, name, w)) return -1;
    }
    return 0;
}

double segDist2(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const double cx = ax + t * dx - px, cy = ay + t * dy - py;
    return cx * cx + cy * cy;
}

}  // namespace

bool buildStreetUrl(const char *apiKey, int z, int x, int y, char *out, size_t n) {
    if (!isSafeToken(apiKey)) return false;
    return fits(snprintf(out, n, "https://api.maptiler.com/tiles/v3-openmaptiles/%d/%d/%d.pbf?key=%s", z, x, y,
                         apiKey),
                n);
}

bool buildStreetPath(const char *root, int z, int x, int y, char *out, size_t n) {
    return fits(snprintf(out, n, "%s/%d/%d/%d.bin", root, z, x, y), n);
}

bool buildStreetDir(const char *root, int z, int x, char *out, size_t n) {
    return fits(snprintf(out, n, "%s/%d/%d", root, z, x), n);
}

int convertMvtStreets(ByteSource &mvt, ByteSink &out) {
    StreetWriter w(out);
    Cursor top(mvt, 0, mvt.size());
    uint32_t num, wire;
    while (!top.done()) {
        if (!top.field(num, wire)) return -1;
        if (num == 3 && wire == 2) {
            size_t s, l;
            if (!top.lenDelim(s, l) || processLayer(mvt, s, l, w) < 0) return -1;
        } else if (!top.skipValue(wire)) {
            return -1;
        }
    }
    if (!w.headerDone() && !w.header(4096)) return -1;
    return w.parts();
}

bool nearestStreet(ByteSource &streets, double fx, double fy, char *name, size_t n, double &dist) {
    uint8_t h[kHeaderSize];
    if (streets.size() < kHeaderSize || !streets.read(0, h, kHeaderSize) || memcmp(h, kMagic, 4) != 0) return false;
    const double extent = double(h[4] | (h[5] << 8));
    if (extent <= 0) return false;
    const double px = fx * extent, py = fy * extent;
    double best = INFINITY;
    Cursor c(streets, kHeaderSize, streets.size());
    auto rd16 = [&](int16_t &v) {
        uint8_t lo, hi;
        if (!c.byte(lo) || !c.byte(hi)) return false;
        v = int16_t(uint16_t(lo | (hi << 8)));
        return true;
    };
    while (!c.done()) {
        uint8_t nl;
        char cur[kMaxName + 1];
        if (!c.byte(nl) || nl > kMaxName) return false;
        for (uint8_t i = 0; i < nl; ++i)
            if (!c.byte(reinterpret_cast<uint8_t &>(cur[i]))) return false;
        cur[nl] = 0;
        int16_t np;
        if (!rd16(np)) return false;
        int16_t ax = 0, ay = 0;
        bool better = false;
        for (int i = 0; i < uint16_t(np); ++i) {
            int16_t bx, by;
            if (!rd16(bx) || !rd16(by)) return false;
            if (i > 0) {
                const double d = segDist2(px, py, ax, ay, bx, by);
                if (d < best) {
                    best = d;
                    better = true;
                }
            }
            ax = bx;
            ay = by;
        }
        if (better && n) snprintf(name, n, "%s", cur);
    }
    if (!isfinite(best)) return false;
    dist = sqrt(best) / extent;
    return true;
}

void asciiFold(const char *utf8, size_t len, char *out, size_t n) {
    static const char *const kLatin1[64] = {
        "A", "A", "A", "A", "A", "A", "AE", "C", "E", "E", "E", "E", "I", "I", "I",  "I",
        "D", "N", "O", "O", "O", "O", "O",  "x", "O", "U", "U", "U", "U", "Y", "Th", "ss",
        "a", "a", "a", "a", "a", "a", "ae", "c", "e", "e", "e", "e", "i", "i", "i",  "i",
        "d", "n", "o", "o", "o", "o", "o",  "/", "o", "u", "u", "u", "u", "y", "th", "y"};
    static const char kExtA[] =
        "AaAaAa" "CcCcCcCc" "DdDd" "EeEeEeEeEe" "GgGgGgGg" "HhHh" "IiIiIiIiIi" "Jj" "Jj" "Kkk" "LlLlLlLlLl"
        "NnNnNnnNn" "OoOoOo" "Oo" "RrRrRr" "SsSsSsSs" "TtTtTt" "UuUuUuUuUuUu" "Ww" "YyY" "ZzZzZz" "s";
    static_assert(sizeof(kExtA) - 1 == 128, "Latin Extended-A table");
    if (!n) return;
    size_t o = 0;
    auto emit = [&](const char *s) {
        for (; *s && o + 1 < n; ++s) out[o++] = *s;
    };
    const uint8_t *p = reinterpret_cast<const uint8_t *>(utf8), *end = p + len;
    while (p < end && o + 1 < n) {
        uint32_t cp;
        int extra;
        if (*p < 0x80) { cp = *p; extra = 0; }
        else if ((*p & 0xe0) == 0xc0) { cp = *p & 0x1f; extra = 1; }
        else if ((*p & 0xf0) == 0xe0) { cp = *p & 0x0f; extra = 2; }
        else if ((*p & 0xf8) == 0xf0) { cp = *p & 0x07; extra = 3; }
        else { ++p; continue; }
        if (end - p <= extra) break;
        ++p;
        bool ok = true;
        for (int i = 0; i < extra; ++i, ++p) {
            if ((*p & 0xc0) != 0x80) { ok = false; break; }
            cp = (cp << 6) | (*p & 0x3f);
        }
        if (!ok) continue;
        char one[3] = {0, 0, 0};
        if (cp >= 0x20 && cp < 0x7f) {
            one[0] = char(cp);
            emit(one);
        } else if (cp >= 0xc0 && cp <= 0xff) {
            emit(kLatin1[cp - 0xc0]);
        } else if (cp >= 0x100 && cp <= 0x17f) {
            if (cp == 0x132) emit("IJ");
            else if (cp == 0x133) emit("ij");
            else if (cp == 0x152) emit("OE");
            else if (cp == 0x153) emit("oe");
            else {
                one[0] = kExtA[cp - 0x100];
                emit(one);
            }
        } else if (cp == 0x2018 || cp == 0x2019) {
            emit("'");
        } else if (cp == 0x2013 || cp == 0x2014) {
            emit("-");
        } else if (cp == 0xa0) {
            emit(" ");
        }
    }
    out[o] = 0;
}

}  // namespace nav
