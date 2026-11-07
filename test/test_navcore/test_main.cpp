#include <geo.h>
#include <gpx.h>
#include <streets.h>
#include <tilesource.h>
#include <unity.h>

#include <string.h>
#include <string>
#include <vector>

using namespace nav;

void setUp() {}
void tearDown() {}

static void test_world_px_roundtrip() {
    const double lat = 47.6062, lon = -122.3321;
    for (int z = 1; z <= 18; ++z) {
        WorldPx p = latLonToWorldPx(lat, lon, z);
        LatLon ll = worldPxToLatLon(p.x, p.y, z);
        TEST_ASSERT_DOUBLE_WITHIN(1e-7, lat, ll.lat);
        TEST_ASSERT_DOUBLE_WITHIN(1e-7, lon, ll.lon);
    }
}

static void test_known_tile() {
    // Seattle at z=10 is OSM tile 164/357.
    WorldPx p = latLonToWorldPx(47.6062, -122.3321, 10);
    TEST_ASSERT_EQUAL_INT(164, int(p.x / kTileSize));
    TEST_ASSERT_EQUAL_INT(357, int(p.y / kTileSize));
    WorldPx origin = latLonToWorldPx(0, 0, 1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 256.0, origin.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 256.0, origin.y);
}

static void test_haversine_and_bearing() {
    LatLon london{51.5074, -0.1278}, paris{48.8566, 2.3522};
    TEST_ASSERT_DOUBLE_WITHIN(1500, 343500, haversineM(london, paris));
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 148.1, initialBearingDeg(london, paris));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, initialBearingDeg({0, 0}, {1, 0}));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, initialBearingDeg({0, 0}, {0, 1}));
    LatLon d = destination(london, 148.1, 343500);
    TEST_ASSERT_DOUBLE_WITHIN(2000, 0, haversineM(d, paris));
}

static void test_bbox_and_tile_count() {
    BBox b = bboxAround({47.6062, -122.3321}, 5000);
    TEST_ASSERT_TRUE(bboxValid(b));
    TEST_ASSERT_DOUBLE_WITHIN(100, 10000, haversineM({b.south, -122.3321}, {b.north, -122.3321}));
    TileRange r = tileRangeFor(b, 1);
    TEST_ASSERT_EQUAL_UINT32(1, r.count());
    TEST_ASSERT_EQUAL_UINT32(1, tileCount(b, 1, 1));
    uint32_t prev = 0;
    for (int z = 8; z <= 16; ++z) {
        uint32_t c = tileRangeFor(b, z).count();
        TEST_ASSERT_TRUE(c >= prev);
        prev = c;
    }
    TEST_ASSERT_EQUAL_UINT32(tileRangeFor(b, 14).count() + tileRangeFor(b, 15).count(), tileCount(b, 14, 15));
    BBox world{-85, -180, 85, 179.9999};
    TEST_ASSERT_EQUAL_UINT32(16, tileRangeFor(world, 2).count());
}

static void test_bbox_antimeridian_clip() {
    BBox b = bboxAround({0, 179.99}, 10000);
    TEST_ASSERT_TRUE(bboxValid(b));
    TEST_ASSERT_TRUE(b.east <= 180.0);
}

static void test_compass_and_format() {
    TEST_ASSERT_EQUAL_STRING("N", compassPoint(359));
    TEST_ASSERT_EQUAL_STRING("NE", compassPoint(44));
    TEST_ASSERT_EQUAL_STRING("S", compassPoint(-180));
    char buf[16];
    formatDistance(950, false, buf, sizeof buf);
    TEST_ASSERT_EQUAL_STRING("950m", buf);
    formatDistance(1234, false, buf, sizeof buf);
    TEST_ASSERT_EQUAL_STRING("1.23km", buf);
    formatDistance(100, true, buf, sizeof buf);
    TEST_ASSERT_EQUAL_STRING("328ft", buf);
    formatDistance(16093.44, true, buf, sizeof buf);
    TEST_ASSERT_EQUAL_STRING("10.0mi", buf);
}

static void test_tile_urls_and_paths() {
    char buf[200];
    TEST_ASSERT_TRUE(buildTileUrl(Provider::MapTiler, "streets-v2", "abc123", 10, 164, 357, buf, sizeof buf));
    TEST_ASSERT_EQUAL_STRING("https://api.maptiler.com/maps/streets-v2/256/10/164/357.jpg?key=abc123", buf);
    TEST_ASSERT_TRUE(buildTileUrl(Provider::Thunderforest, "outdoors", "k", 3, 1, 2, buf, sizeof buf));
    TEST_ASSERT_EQUAL_STRING("https://tile.thunderforest.com/outdoors/3/1/2.jpg90?apikey=k", buf);
    TEST_ASSERT_FALSE(buildTileUrl(Provider::MapTiler, "streets", "bad key&x=1", 1, 0, 0, buf, sizeof buf));
    TEST_ASSERT_FALSE(buildTileUrl(Provider::MapTiler, "../etc", "k", 1, 0, 0, buf, sizeof buf));
    TEST_ASSERT_FALSE(buildTileUrl(Provider::MapTiler, "s", "k", 1, 2, 0, buf, sizeof buf));
    TEST_ASSERT_TRUE(buildTilePath("/gpsnav/tiles", Provider::MapTiler, "streets-v2", 5, 3, 4, buf, sizeof buf));
    TEST_ASSERT_EQUAL_STRING("/gpsnav/tiles/maptiler-streets-v2/5/3/4.jpg", buf);
    TEST_ASSERT_FALSE(buildTilePath("/gpsnav/tiles", Provider::MapTiler, "streets-v2", 5, 3, 4, buf, 10));
    Provider p;
    TEST_ASSERT_TRUE(parseProvider("thunderforest", p));
    TEST_ASSERT_TRUE(p == Provider::Thunderforest);
    TEST_ASSERT_FALSE(parseProvider("osm", p));
}

struct Pt {
    GpxPointKind kind;
    double lat, lon;
    std::string name;
};

static void collect(void *ctx, GpxPointKind kind, double lat, double lon, const char *name) {
    static_cast<std::vector<Pt> *>(ctx)->push_back({kind, lat, lon, name});
}

static void test_gpx_parse() {
    const char *doc =
        "<?xml version=\"1.0\"?>\n<gpx version=\"1.1\">\n"
        "<wpt lat=\"47.1\" lon=\"-122.5\"><ele>3</ele><name>Camp &amp; Lake</name></wpt>\n"
        "<wpt lon='-1.5' lat='2.25'/>\n"
        "<rte><rtept lat=\"10\" lon=\"20\"></rtept><rtept lat=\"11\" lon=\"21\"/></rte>\n"
        "<trk><name>t</name><trkseg>\n<trkpt\n  lat=\"-33.5\" lon=\"151.25\"><ele>1</ele></trkpt>\n"
        "<trkpt lat=\"-33.6\" lon=\"151.26\"/></trkseg></trk></gpx>";
    std::vector<Pt> pts;
    GpxParser parser(collect, &pts);
    // Feed in small chunks to exercise state carried across calls.
    for (size_t i = 0, n = strlen(doc); i < n; i += 7) parser.feed(doc + i, (n - i) < 7 ? n - i : 7);
    TEST_ASSERT_EQUAL_UINT32(6, pts.size());
    TEST_ASSERT_TRUE(pts[0].kind == GpxPointKind::Waypoint);
    TEST_ASSERT_EQUAL_STRING("Camp &amp; Lake", pts[0].name.c_str());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -122.5, pts[0].lon);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.25, pts[1].lat);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -1.5, pts[1].lon);
    TEST_ASSERT_TRUE(pts[2].kind == GpxPointKind::Route);
    TEST_ASSERT_TRUE(pts[4].kind == GpxPointKind::Track);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -33.5, pts[4].lat);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 151.26, pts[5].lon);
}

static void test_gpx_write_roundtrip() {
    std::string doc;
    char buf[256];
    gpxTrackHeader(buf, sizeof buf, "Morning <ride>");
    doc += buf;
    gpxTrackPoint(buf, sizeof buf, 47.1234567, -122.7654321, 12.3, "2026-11-06T12:00:00Z");
    doc += buf;
    doc += kGpxTrackFooter;
    TEST_ASSERT_NOT_NULL(strstr(doc.c_str(), "Morning &lt;ride&gt;"));
    std::vector<Pt> pts;
    GpxParser parser(collect, &pts);
    parser.feed(doc.c_str(), doc.size());
    TEST_ASSERT_EQUAL_UINT32(1, pts.size());
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 47.1234567, pts[0].lat);
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, -122.7654321, pts[0].lon);
}

using Bytes = std::vector<uint8_t>;
static void pbVarint(Bytes &b, uint64_t v) {
    while (v >= 0x80) {
        b.push_back(uint8_t(v | 0x80));
        v >>= 7;
    }
    b.push_back(uint8_t(v));
}
static void pbKey(Bytes &b, uint32_t num, uint32_t wire) { pbVarint(b, (num << 3) | wire); }
static void pbBytes(Bytes &b, uint32_t num, const Bytes &d) {
    pbKey(b, num, 2);
    pbVarint(b, d.size());
    b.insert(b.end(), d.begin(), d.end());
}
static void pbStr(Bytes &b, uint32_t num, const char *s) { pbBytes(b, num, Bytes(s, s + strlen(s))); }
static uint32_t zz(int32_t v) { return uint32_t(v << 1) ^ uint32_t(v >> 31); }
static Bytes packed(std::initializer_list<uint32_t> v) {
    Bytes b;
    for (auto x : v) pbVarint(b, x);
    return b;
}
static Bytes feature(uint64_t type, const Bytes &tags, const Bytes &geom) {
    Bytes f;
    pbBytes(f, 2, tags);
    pbKey(f, 3, 0);
    pbVarint(f, type);
    pbBytes(f, 4, geom);
    return f;
}
static void pbValue(Bytes &layer, const char *s) {
    Bytes v;
    pbStr(v, 1, s);
    pbBytes(layer, 4, v);
}
struct VecSink : nav::ByteSink {
    Bytes d;
    bool write(const uint8_t *b, size_t n) override {
        d.insert(d.end(), b, b + n);
        return true;
    }
};

static Bytes sampleTile() {
    Bytes water;
    pbStr(water, 1, "water");
    pbBytes(water, 2, feature(3, packed({0, 0}), packed({9, zz(1), zz(1), 18, zz(5), zz(0), zz(0), zz(5), 15})));
    pbStr(water, 3, "name");
    pbValue(water, "Lake");
    pbKey(water, 5, 0);
    pbVarint(water, 4096);

    Bytes tn;
    pbStr(tn, 1, "transportation_name");
    pbBytes(tn, 2, feature(2, packed({0, 0, 1, 1}), packed({9, zz(100), zz(100), 18, zz(400), zz(0), zz(0), zz(400)})));
    pbBytes(tn, 2, feature(2, packed({1, 2}), packed({9, zz(1000), zz(1000), 10, zz(0), zz(1000)})));
    pbBytes(tn, 2, feature(1, packed({1, 1}), packed({9, zz(5), zz(5)})));
    pbBytes(tn, 2, feature(2, packed({0, 0}), packed({9, zz(10), zz(10), 10, zz(50), zz(0)})));
    pbBytes(tn, 2, feature(2, packed({1, 3, 2, 4}), packed({9, zz(3000), zz(3000), 10, zz(500), zz(0)})));
    pbStr(tn, 3, "class");
    pbStr(tn, 3, "name");
    pbStr(tn, 3, "name:latin");
    pbValue(tn, "primary");
    pbValue(tn, "Main Street");
    pbValue(tn, "Caf\xc3\xa9 \xc3\x9cnter");
    pbValue(tn, "\xd0\x9c\xd0\xbe\xd1\x81\xd0\xba\xd0\xb2\xd0\xb0");
    pbValue(tn, "Moskva");
    pbKey(tn, 5, 0);
    pbVarint(tn, 4096);

    Bytes tile;
    pbBytes(tile, 3, water);
    pbBytes(tile, 3, tn);
    return tile;
}

void test_mvt_streets() {
    const Bytes tile = sampleTile();
    nav::MemSource src(tile.data(), tile.size());
    VecSink out;
    TEST_ASSERT_EQUAL_INT(3, nav::convertMvtStreets(src, out));

    nav::MemSource streets(out.d.data(), out.d.size());
    char name[64];
    double dist = 0;
    TEST_ASSERT_TRUE(nav::nearestStreet(streets, 300 / 4096.0, 110 / 4096.0, name, sizeof name, dist));
    TEST_ASSERT_EQUAL_STRING("Main Street", name);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 10 / 4096.0, dist);
    TEST_ASSERT_TRUE(nav::nearestStreet(streets, 1010 / 4096.0, 1500 / 4096.0, name, sizeof name, dist));
    TEST_ASSERT_EQUAL_STRING("Cafe Unter", name);
    TEST_ASSERT_TRUE(nav::nearestStreet(streets, 3200 / 4096.0, 3010 / 4096.0, name, sizeof name, dist));
    TEST_ASSERT_EQUAL_STRING("Moskva", name);

    VecSink bad;
    nav::MemSource truncated(tile.data(), tile.size() - 3);
    TEST_ASSERT_EQUAL_INT(-1, nav::convertMvtStreets(truncated, bad));

    VecSink empty;
    nav::MemSource none(tile.data(), 0);
    TEST_ASSERT_EQUAL_INT(0, nav::convertMvtStreets(none, empty));
    nav::MemSource emptyStreets(empty.d.data(), empty.d.size());
    TEST_ASSERT_FALSE(nav::nearestStreet(emptyStreets, 0.5, 0.5, name, sizeof name, dist));
}

void test_ascii_fold_and_street_paths() {
    char out[32];
    const char *s1 = "Stra\xc3\x9f" "e";
    nav::asciiFold(s1, strlen(s1), out, sizeof out);
    TEST_ASSERT_EQUAL_STRING("Strasse", out);
    const char *s2 = "\xc5\x81\xc3\xb3" "d\xc5\xba";
    nav::asciiFold(s2, strlen(s2), out, sizeof out);
    TEST_ASSERT_EQUAL_STRING("Lodz", out);
    const char *s3 = "Rue de l\xe2\x80\x99" "Eglise";
    nav::asciiFold(s3, strlen(s3), out, sizeof out);
    TEST_ASSERT_EQUAL_STRING("Rue de l'Eglise", out);
    nav::asciiFold("abcdef", 6, out, 4);
    TEST_ASSERT_EQUAL_STRING("abc", out);

    char buf[160];
    TEST_ASSERT_TRUE(nav::buildStreetUrl("abc", 14, 1, 2, buf, sizeof buf));
    TEST_ASSERT_EQUAL_STRING("https://api.maptiler.com/tiles/v3-openmaptiles/14/1/2.pbf?key=abc", buf);
    TEST_ASSERT_FALSE(nav::buildStreetUrl("a/b", 14, 1, 2, buf, sizeof buf));
    TEST_ASSERT_TRUE(nav::buildStreetPath("/gpsnav/streets", 14, 1, 2, buf, sizeof buf));
    TEST_ASSERT_EQUAL_STRING("/gpsnav/streets/14/1/2.bin", buf);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_world_px_roundtrip);
    RUN_TEST(test_known_tile);
    RUN_TEST(test_haversine_and_bearing);
    RUN_TEST(test_bbox_and_tile_count);
    RUN_TEST(test_bbox_antimeridian_clip);
    RUN_TEST(test_compass_and_format);
    RUN_TEST(test_tile_urls_and_paths);
    RUN_TEST(test_gpx_parse);
    RUN_TEST(test_gpx_write_roundtrip);
    RUN_TEST(test_mvt_streets);
    RUN_TEST(test_ascii_fold_and_street_paths);
    return UNITY_END();
}
