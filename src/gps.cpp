#include "gps.h"

namespace {
// Cap LoRa-1262 GNSS (ATGM336H-6N) on Cardputer ADV: module TX -> G15, module RX <- G13.
constexpr int kGpsRxPin = 15;
constexpr int kGpsTxPin = 13;
constexpr uint32_t kGpsBaud = 115200;
constexpr uint32_t kStaleMs = 5000;
HardwareSerial gpsSerial(1);
}  // namespace

void GpsReceiver::begin() {
    gpsSerial.setRxBufferSize(2048);
    gpsSerial.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
}

void GpsReceiver::poll() {
    while (gpsSerial.available() > 0) gps_.encode(char(gpsSerial.read()));
    if (gps_.charsProcessed() != lastChars_) {
        lastChars_ = gps_.charsProcessed();
        lastCharAt_ = millis();
    }
}

bool GpsReceiver::receivingData() const { return lastChars_ > 0 && millis() - lastCharAt_ < 3000; }

GpsFix GpsReceiver::fix() const {
    GpsFix f;
    f.ageMs = gps_.location.age();
    f.valid = gps_.location.isValid() && f.ageMs < kStaleMs;
    f.lat = gps_.location.lat();
    f.lon = gps_.location.lng();
    f.altM = gps_.altitude.isValid() ? gps_.altitude.meters() : 0;
    f.speedKmh = gps_.speed.isValid() ? gps_.speed.kmph() : 0;
    f.courseValid = gps_.course.isValid() && f.speedKmh > 2.0;
    f.courseDeg = gps_.course.deg();
    f.sats = gps_.satellites.isValid() ? gps_.satellites.value() : 0;
    f.hdop = gps_.hdop.isValid() ? gps_.hdop.hdop() : 0;
    return f;
}

bool GpsReceiver::isoTime(char *buf, size_t n) const {
    if (!gps_.date.isValid() || !gps_.time.isValid() || gps_.date.year() < 2020) return false;
    snprintf(buf, n, "%04u-%02u-%02uT%02u:%02u:%02uZ", gps_.date.year(), gps_.date.month(), gps_.date.day(),
             gps_.time.hour(), gps_.time.minute(), gps_.time.second());
    return true;
}

bool GpsReceiver::fileStamp(char *buf, size_t n) const {
    if (!gps_.date.isValid() || !gps_.time.isValid() || gps_.date.year() < 2020) return false;
    snprintf(buf, n, "%04u%02u%02u-%02u%02u%02u", gps_.date.year(), gps_.date.month(), gps_.date.day(),
             gps_.time.hour(), gps_.time.minute(), gps_.time.second());
    return true;
}
