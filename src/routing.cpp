#include "routing.h"

#include <SD.h>
#include <roads.h>
#include <streets.h>

#include "app.h"
#include "input.h"
#include "storage.h"
#include "ui.h"

using app::canvas;

namespace routing {

namespace {

// Road file with a one-block read cache: the router does many small reads close together.
class BlockSource : public nav::ByteSource {
public:
    static constexpr size_t kBlock = 512;
    File f;
    void reset() {
        if (f) f.close();
        cached_ = SIZE_MAX;
        got_ = 0;
    }
    size_t size() override { return f.size(); }
    bool read(size_t off, uint8_t *buf, size_t n) override {
        while (n) {
            const size_t blk = off / kBlock * kBlock;
            if (blk != cached_) {
                cached_ = SIZE_MAX;
                if (!f.seek(blk)) return false;
                const int r = f.read(cache_, kBlock);
                if (r <= 0) return false;
                got_ = size_t(r);
                cached_ = blk;
            }
            const size_t o = off - blk;
            if (o >= got_) return false;
            const size_t k = min(n, got_ - o);
            memcpy(buf, cache_ + o, k);
            buf += k;
            off += k;
            n -= k;
        }
        return true;
    }

private:
    uint8_t cache_[kBlock];
    size_t cached_ = SIZE_MAX, got_ = 0;
};

class FileTiles : public nav::RoadTiles {
public:
    ~FileTiles() {
        for (auto &s : slots_) s.src.reset();
    }
    nav::ByteSource *tile(int tx, int ty) override {
        for (auto &s : slots_)
            if (s.used && s.tx == tx && s.ty == ty) {
                s.age = ++clock_;
                return s.ok ? &s.src : nullptr;
            }
        Slot *v = &slots_[0];
        for (auto &s : slots_) {
            if (!s.used) {
                v = &s;
                break;
            }
            if (s.age < v->age) v = &s;
        }
        v->src.reset();
        v->used = true;
        v->tx = tx;
        v->ty = ty;
        v->age = ++clock_;
        char path[96];
        v->ok = nav::buildRoadPath(storage::kRoadsRoot, nav::kStreetZoom, tx, ty, path, sizeof path) &&
                SD.exists(path) && (v->src.f = SD.open(path, FILE_READ));
        return v->ok ? &v->src : nullptr;
    }

private:
    struct Slot {
        bool used = false, ok = false;
        int tx = 0, ty = 0;
        uint32_t age = 0;
        BlockSource src;
    };
    Slot slots_[4];  // SD allows 5 open files
    uint32_t clock_ = 0;
};

struct Progress {
    const char *mode;
    uint32_t lastDraw = 0;
};

void drawProgress(const char *mode, uint32_t visited) {
    canvas.fillScreen(ui::kBg);
    ui::header("Routing", mode);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(ui::kFg);
    canvas.drawString("Working out the route on the", 4, 30);
    canvas.drawString("downloaded streets...", 4, 42);
    char line[48];
    snprintf(line, sizeof line, "%lu junctions searched", (unsigned long)visited);
    canvas.setTextColor(ui::kDim);
    canvas.drawString(line, 4, 60);
    ui::footer("` cancel");
    ui::present();
}

bool poll(void *ctx, uint32_t visited) {
    auto *p = static_cast<Progress *>(ctx);
    app::service();
    if (millis() - p->lastDraw > 250) {
        p->lastDraw = millis();
        drawProgress(p->mode, visited);
    }
    return readKey().back;
}

}  // namespace

bool compute(const nav::LatLon &from, const nav::LatLon &to, String &msg) {
    const bool walk = app::config.walkRoutes;
    Progress prog{walk ? "Walk" : "Drive"};
    drawProgress(prog.mode, 0);

    nav::RouteOptions opt;
    opt.mode = walk ? nav::RouteMode::Walk : nav::RouteMode::Drive;
    // Search budget from free memory: ~48 bytes per junction, the largest single block ~16.
    constexpr uint32_t kReserve = 40 * 1024;
    const uint32_t freeB = ESP.getFreeHeap(), block = ESP.getMaxAllocHeap();
    uint32_t n = freeB > kReserve ? (freeB - kReserve) / 48 : 0;
    n = min(n, block / 20);
    opt.maxNodes = constrain(n, 300u, 12000u);
    opt.poll = &poll;
    opt.ctx = &prog;

    nav::RouteResult r;
    nav::RouteStatus st;
    {
        FileTiles tiles;
        st = nav::findRoute(tiles, from, to, opt, r);
    }
    log_i("route %d visited %u budget %u free %u", int(st), r.visited, opt.maxNodes, ESP.getFreeHeap());
    if (st != nav::RouteStatus::Ok) {
        msg = nav::routeStatusText(st);
        if (st == nav::RouteStatus::NoData || st == nav::RouteStatus::Error)
            msg += "\n\nRegions downloaded before v0.1.7 need one re-run of the download (only the street "
                   "data is fetched again).";
        else if (st == nav::RouteStatus::NoRoute && !walk)
            msg += "\n\nDrive mode follows one-way streets and skips footpaths; try Walk in Settings.";
        return false;
    }
    char len[16];
    nav::formatDistance(r.lengthM, app::config.imperial, len, sizeof len);
    const uint32_t mins = uint32_t(r.timeS / 60 + 0.5);
    msg = String(walk ? "Walk " : "Drive ") + len + ", ~" +
          (mins < 60 ? String(mins ? mins : 1) + " min" : String(mins / 60) + " h " + String(mins % 60) + " min");
    app::navdata.setRoute(std::move(r.pts), walk ? "Walk route" : "Drive route");
    return true;
}

}  // namespace routing
