#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GpsFix {
    bool valid = false;  // location valid and recent
    double lat = 0, lon = 0;
    double altM = 0;
    double speedKmh = 0;
    double courseDeg = 0;
    bool courseValid = false;
    uint32_t sats = 0;
    double hdop = 0;
    uint32_t ageMs = 0;
};

class GpsReceiver {
public:
    void begin();
    void poll();
    GpsFix fix() const;
    bool receivingData() const;
    uint32_t charsProcessed() const { return gps_.charsProcessed(); }
    uint32_t failedChecksums() const { return gps_.failedChecksum(); }
    // UTC time as ISO-8601 (2026-11-06T12:00:00Z); false if unknown.
    bool isoTime(char *buf, size_t n) const;
    // UTC time as YYYYMMDD-HHMMSS for file names; false if unknown.
    bool fileStamp(char *buf, size_t n) const;

private:
    mutable TinyGPSPlus gps_;
    uint32_t lastCharAt_ = 0;
    uint32_t lastChars_ = 0;
};
