#include "storage.h"

#include <SD.h>
#include <SPI.h>

namespace storage {

namespace {
constexpr int kSdSck = 40;
constexpr int kSdMiso = 39;
constexpr int kSdMosi = 14;
constexpr int kSdCs = 12;
// The Cap LoRa-1262 radio shares the SD SPI bus; keep it deselected.
constexpr int kLoraCs = 5;
}  // namespace

bool begin() {
    pinMode(kLoraCs, OUTPUT);
    digitalWrite(kLoraCs, HIGH);
    SPI.begin(kSdSck, kSdMiso, kSdMosi, kSdCs);
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (SD.begin(kSdCs, SPI, 25000000)) {
            mkdirs(kTilesRoot);
            mkdirs(kStreetsRoot);
            mkdirs(kRoutesDir);
            mkdirs(kTracksDir);
            return true;
        }
        delay(200);
    }
    return false;
}

bool mkdirs(const char *path) {
    if (SD.exists(path)) return true;
    char buf[128];
    strlcpy(buf, path, sizeof(buf));
    for (char *p = buf + 1; *p; ++p) {
        if (*p != '/') continue;
        *p = 0;
        if (!SD.exists(buf)) SD.mkdir(buf);
        *p = '/';
    }
    return SD.mkdir(buf) || SD.exists(buf);
}

uint64_t freeBytes() { return SD.totalBytes() - SD.usedBytes(); }
uint64_t totalBytes() { return SD.totalBytes(); }

}  // namespace storage
