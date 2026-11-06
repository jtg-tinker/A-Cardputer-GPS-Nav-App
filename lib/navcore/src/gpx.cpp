#include "gpx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace nav {

const char kGpxTrackFooter[] = "</trkseg></trk></gpx>\n";

void GpxParser::feed(const char *data, size_t len) {
    for (size_t i = 0; i < len; ++i) feed(data[i]);
}

void GpxParser::feed(char c) {
    if (inTag_) {
        if (c == '>') {
            tag_[tagLen_] = 0;
            inTag_ = false;
            onTag();
        } else if (tagLen_ + 1 < sizeof(tag_)) {
            tag_[tagLen_++] = c;
        }
        return;
    }
    if (c == '<') {
        inTag_ = true;
        tagLen_ = 0;
        return;
    }
    if (inName_ && nameLen_ + 1 < sizeof(name_) && c != '\r' && c != '\n') {
        name_[nameLen_++] = c;
        name_[nameLen_] = 0;
    }
}

static bool tagIs(const char *tag, const char *name) {
    const size_t n = strlen(name);
    if (strncmp(tag, name, n) != 0) return false;
    const char next = tag[n];
    return next == 0 || next == ' ' || next == '\t' || next == '\r' || next == '\n' || next == '/';
}

bool GpxParser::attr(const char *tag, const char *name, double &out) {
    const size_t n = strlen(name);
    for (const char *p = tag; (p = strstr(p, name)) != nullptr; p += n) {
        const char prev = p == tag ? ' ' : p[-1];
        if (prev != ' ' && prev != '\t' && prev != '\r' && prev != '\n') continue;
        const char *q = p + n;
        while (*q == ' ') ++q;
        if (*q != '=') continue;
        ++q;
        while (*q == ' ') ++q;
        if (*q != '"' && *q != '\'') continue;
        char *end = nullptr;
        out = strtod(q + 1, &end);
        return end && end != q + 1;
    }
    return false;
}

void GpxParser::onTag() {
    const char *t = tag_;
    if (*t == '?' || *t == '!') return;
    const bool closing = *t == '/';
    if (closing) ++t;
    const bool selfClosing = tagLen_ > 0 && tag_[tagLen_ - 1] == '/';

    if (closing) {
        if (tagIs(t, "name")) inName_ = false;
        if (tagIs(t, "wpt") && inWpt_) {
            inWpt_ = false;
            cb_(ctx_, GpxPointKind::Waypoint, wptLat_, wptLon_, name_);
        }
        return;
    }

    double lat, lon;
    if (tagIs(t, "trkpt") || tagIs(t, "rtept")) {
        if (attr(t, "lat", lat) && attr(t, "lon", lon))
            cb_(ctx_, tagIs(t, "trkpt") ? GpxPointKind::Track : GpxPointKind::Route, lat, lon, "");
    } else if (tagIs(t, "wpt")) {
        if (attr(t, "lat", lat) && attr(t, "lon", lon)) {
            name_[0] = 0;
            nameLen_ = 0;
            if (selfClosing) {
                cb_(ctx_, GpxPointKind::Waypoint, lat, lon, "");
            } else {
                inWpt_ = true;
                wptLat_ = lat;
                wptLon_ = lon;
            }
        }
    } else if (tagIs(t, "name") && inWpt_ && !selfClosing) {
        inName_ = true;
        nameLen_ = 0;
        name_[0] = 0;
    }
}

static void xmlEscape(const char *in, char *out, size_t n) {
    size_t o = 0;
    for (; in && *in && o + 7 < n; ++in) {
        switch (*in) {
            case '&': o += snprintf(out + o, n - o, "&amp;"); break;
            case '<': o += snprintf(out + o, n - o, "&lt;"); break;
            case '>': o += snprintf(out + o, n - o, "&gt;"); break;
            case '"': o += snprintf(out + o, n - o, "&quot;"); break;
            default: out[o++] = *in;
        }
    }
    out[o] = 0;
}

int gpxTrackHeader(char *out, size_t n, const char *name) {
    char esc[96];
    xmlEscape(name, esc, sizeof(esc));
    return snprintf(out, n,
                    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                    "<gpx version=\"1.1\" creator=\"cardputer-gps-nav\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
                    "<trk><name>%s</name><trkseg>\n",
                    esc);
}

int gpxTrackPoint(char *out, size_t n, double lat, double lon, double eleM, const char *isoTime) {
    return snprintf(out, n, "<trkpt lat=\"%.7f\" lon=\"%.7f\"><ele>%.1f</ele><time>%s</time></trkpt>\n", lat, lon,
                    eleM, isoTime ? isoTime : "");
}

int gpxWaypoint(char *out, size_t n, double lat, double lon, const char *name) {
    char esc[96];
    xmlEscape(name, esc, sizeof(esc));
    return snprintf(out, n, "<wpt lat=\"%.7f\" lon=\"%.7f\"><name>%s</name></wpt>\n", lat, lon, esc);
}

}  // namespace nav
