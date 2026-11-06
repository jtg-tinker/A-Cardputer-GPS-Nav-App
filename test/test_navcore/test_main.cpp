#include <geo.h>
#include <gpx.h>
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
    return UNITY_END();
}
