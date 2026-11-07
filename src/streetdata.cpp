#include "streetdata.h"

#include <SD.h>
#include <math.h>
#include <miniz.h>
#include <streets.h>

#include <memory>
#include <new>

#include "storage.h"

namespace streetdata {

namespace {

constexpr double kMaxStreetDistM = 50;
constexpr size_t kMaxConvertInMemory = 96 * 1024;
constexpr size_t kMaxLookupInMemory = 24 * 1024;

class FileSource : public nav::ByteSource {
public:
    explicit FileSource(File &f) : f_(f) {}
    size_t size() override { return f_.size(); }
    bool read(size_t offset, uint8_t *buf, size_t n) override {
        return f_.seek(offset) && f_.read(buf, n) == int(n);
    }

private:
    File &f_;
};

class FileSink : public nav::ByteSink {
public:
    explicit FileSink(File &f) : f_(f) {}
    bool write(const uint8_t *buf, size_t n) override { return f_.write(buf, n) == n; }

private:
    File &f_;
};

bool isGzip(File &f) {
    uint8_t m[2];
    const bool gz = f.read(m, 2) == 2 && m[0] == 0x1f && m[1] == 0x8b;
    f.seek(0);
    return gz;
}

bool gunzip(File &in, File &out) {
    uint8_t h[10];
    if (in.read(h, 10) != 10 || h[2] != 8) return false;
    const uint8_t flg = h[3];
    if (flg & 4) {
        uint8_t x[2];
        if (in.read(x, 2) != 2 || !in.seek(in.position() + (x[0] | (x[1] << 8)))) return false;
    }
    for (uint8_t bit : {uint8_t(8), uint8_t(16)}) {
        if (!(flg & bit)) continue;
        int c;
        do c = in.read();
        while (c > 0);
        if (c < 0) return false;
    }
    if ((flg & 2) && !in.seek(in.position() + 2)) return false;

    std::unique_ptr<tinfl_decompressor> d(new (std::nothrow) tinfl_decompressor);
    std::unique_ptr<uint8_t[]> dict(new (std::nothrow) uint8_t[TINFL_LZ_DICT_SIZE]);
    std::unique_ptr<uint8_t[]> ibuf(new (std::nothrow) uint8_t[1024]);
    if (!d || !dict || !ibuf) return false;
    tinfl_init(d.get());
    size_t inPos = 0, inLen = 0, dictOfs = 0;
    for (;;) {
        if (inPos == inLen && in.available()) {
            const int r = in.read(ibuf.get(), 1024);
            if (r <= 0) return false;
            inLen = size_t(r);
            inPos = 0;
        }
        size_t inBytes = inLen - inPos, outBytes = TINFL_LZ_DICT_SIZE - dictOfs;
        const tinfl_status st = tinfl_decompress(d.get(), ibuf.get() + inPos, &inBytes, dict.get(),
                                                 dict.get() + dictOfs, &outBytes,
                                                 in.available() ? TINFL_FLAG_HAS_MORE_INPUT : 0);
        inPos += inBytes;
        if (outBytes && out.write(dict.get() + dictOfs, outBytes) != outBytes) return false;
        dictOfs = (dictOfs + outBytes) & (TINFL_LZ_DICT_SIZE - 1);
        if (st == TINFL_STATUS_DONE) return true;
        if (st < TINFL_STATUS_DONE) return false;
        if (st == TINFL_STATUS_NEEDS_MORE_INPUT && inPos == inLen && !in.available()) return false;
    }
}

bool convertMvt(File &in, const char *binPath) {
    const String tmp = String(binPath) + ".part";
    File out = SD.open(tmp, FILE_WRITE);
    if (!out) return false;
    FileSink sink(out);
    int parts;
    const size_t sz = in.size();
    std::unique_ptr<uint8_t[]> mem(sz <= kMaxConvertInMemory ? new (std::nothrow) uint8_t[sz ? sz : 1] : nullptr);
    if (mem && in.seek(0) && in.read(mem.get(), sz) == int(sz)) {
        nav::MemSource src(mem.get(), sz);
        parts = nav::convertMvtStreets(src, sink);
    } else {
        mem.reset();
        FileSource src(in);
        parts = nav::convertMvtStreets(src, sink);
    }
    out.close();
    if (parts < 0) {
        SD.remove(tmp);
        return false;
    }
    SD.remove(binPath);
    if (SD.rename(tmp, binPath)) return true;
    SD.remove(tmp);
    return false;
}

struct Cache {
    int tx = -1, ty = -1;
    bool exists = false;
    char path[96] = {0};
    std::unique_ptr<uint8_t[]> data;
    size_t len = 0;
    bool queried = false;
    double fx = 0, fy = 0;
    int result = -1;
    char name[64] = {0};
};
Cache cache;

void loadTile(int tx, int ty) {
    cache = Cache();
    cache.tx = tx;
    cache.ty = ty;
    if (!nav::buildStreetPath(storage::kStreetsRoot, nav::kStreetZoom, tx, ty, cache.path, sizeof cache.path)) return;
    File f = SD.open(cache.path, FILE_READ);
    if (!f) return;
    cache.exists = true;
    const size_t sz = f.size();
    if (sz <= kMaxLookupInMemory) {
        cache.data.reset(new (std::nothrow) uint8_t[sz ? sz : 1]);
        if (cache.data && f.read(cache.data.get(), sz) == int(sz)) cache.len = sz;
        else cache.data.reset();
    }
    f.close();
}

}  // namespace

bool convertDownloaded(const char *pbfPath, const char *binPath) {
    File in = SD.open(pbfPath, FILE_READ);
    if (!in) return false;
    bool ok;
    if (isGzip(in)) {
        const String raw = String(pbfPath) + ".raw";
        File r = SD.open(raw, FILE_WRITE);
        ok = r && gunzip(in, r);
        if (r) r.close();
        in.close();
        if (ok) {
            File rr = SD.open(raw, FILE_READ);
            ok = rr && convertMvt(rr, binPath);
            if (rr) rr.close();
        }
        SD.remove(raw);
    } else {
        ok = convertMvt(in, binPath);
        in.close();
    }
    invalidate();
    return ok;
}

int lookup(const nav::LatLon &p, char *name, size_t n) {
    if (n) name[0] = 0;
    const nav::WorldPx w = nav::latLonToWorldPx(p.lat, p.lon, nav::kStreetZoom);
    const int tx = int(floor(w.x / 256)), ty = int(floor(w.y / 256));
    const double fx = w.x / 256 - tx, fy = w.y / 256 - ty;
    if (tx != cache.tx || ty != cache.ty) loadTile(tx, ty);
    if (!cache.exists) return -1;
    if (!cache.queried || fabs(fx - cache.fx) > 1e-3 || fabs(fy - cache.fy) > 1e-3) {
        char found[64] = {0};
        double dist = 0;
        bool hit;
        if (cache.data) {
            nav::MemSource src(cache.data.get(), cache.len);
            hit = nav::nearestStreet(src, fx, fy, found, sizeof found, dist);
        } else {
            File f = SD.open(cache.path, FILE_READ);
            if (!f) return -1;
            FileSource src(f);
            hit = nav::nearestStreet(src, fx, fy, found, sizeof found, dist);
            f.close();
        }
        const double tileM = 2 * M_PI * nav::kEarthRadiusM * cos(p.lat * DEG_TO_RAD) / double(1 << nav::kStreetZoom);
        cache.queried = true;
        cache.fx = fx;
        cache.fy = fy;
        cache.result = hit && dist * tileM <= kMaxStreetDistM ? 1 : 0;
        snprintf(cache.name, sizeof cache.name, "%s", cache.result ? found : "");
    }
    if (n) snprintf(name, n, "%s", cache.name);
    return cache.result;
}

void invalidate() { cache = Cache(); }

}  // namespace streetdata
