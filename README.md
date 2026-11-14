# Cardputer GPS Nav

Offline moving-map GPS navigator for the **M5Stack Cardputer ADV** with the **Cap LoRa-1262 (GNSS)**.
You download map regions over WiFi on the device itself. Tiles are saved to the microSD card, so the map works offline afterwards.

## Requirements

- M5Stack Cardputer ADV (ESP32-S3, 8 MB flash)
- M5Stack Cap LoRa-1262 (ATGM336H GNSS, NMEA at 115200 baud on G15 RX / G13 TX)
- microSD card, FAT32
- A free API key from [MapTiler](https://cloud.maptiler.com/account/keys/) or [Thunderforest](https://manage.thunderforest.com/)

## Installing

- **M5Burner:** flash `cardputer-gps-nav-m5burner.bin` (full image, offset `0x0`).
- **esptool:** `esptool.py --chip esp32s3 write_flash 0x0 cardputer-gps-nav-m5burner.bin`
- **M5Launcher / OTA / SD:** use `cardputer-gps-nav-app.bin` (app only, `0x10000`).

If flashing won't start, hold **G0** while you plug in USB to enter download mode.

## First run

1. Insert the microSD card and attach the LoRa-1262 cap. Go outside to get a GPS fix.
2. Open **Menu → Settings**. Choose a tile provider, enter your API key, then set up WiFi.
3. Open **Menu → Download maps**. Choose an area: around your GPS position, around the map center, a place search (Nominatim), or coordinates you enter. Pick a radius (1–100 km) and a max detail level (z10–z18; z16 shows streets). The screen shows the tile count and estimated size before the download starts.

   With a MapTiler key the download also fetches MapTiler street data (vector tiles at z14) for the same area. Afterwards the top bar gets a second line showing the nearest named street (within 50 m) at your GPS position, or at the crosshair while you pan. A "-" means there is street data but no named street within 25 m. From z16 up, the street-data lines are drawn on the map in thin green, and the street named in the bar is highlighted in yellow. Thunderforest downloads don't include street data.

   A "~" after the zoom level (e.g. `Z16~`) means part of the screen is an enlarged lower-zoom tile because that tile wasn't downloaded, so small streets may be missing there.
4. If a download is interrupted, open **Download maps → Saved regions** and choose resume. Tiles you already have are skipped.

## Map keys

| Key | Action |
| --- | --- |
| `;` `.` `,` `/` | Pan up / down / left / right |
| `=` / `-` | Zoom in / out |
| `space` or `f` | Follow GPS again |
| `Enter` / `m` / G0 | Main menu |
| `d` | Download maps |
| `g` | Waypoints / go-to |
| `w` | Save a waypoint at the map center |
| `r` | Start / stop track recording |
| `x` | Clear navigation |
| `i` | GPS status |
| `h` / `?` | Help |

In menus, `` ` `` goes back. In text fields, use `fn` plus the key for the arrows and escape.

## SD card layout

```
/gpsnav/config.json         settings (WiFi, provider, API key, units, brightness)
/gpsnav/tiles/<provider>-<style>/<z>/<x>/<y>.jpg
/gpsnav/streets/14/<x>/<y>.bin  street names + lines (MapTiler regions)
/gpsnav/regions.csv         saved download regions
/gpsnav/waypoints.gpx       waypoints
/gpsnav/routes/*.gpx        GPX routes/tracks to follow (copy them here)
/gpsnav/tracks/*.gpx        recorded tracks
```

## Building

```
pip install platformio
pio test -e native          # navcore unit tests (geo math, tile URLs, GPX)
pio run -e cardputer-adv    # firmware -> dist/
```

To regenerate the HTTPS root certificates in `src/certs.h`, run `scripts/gen_certs.sh`.

## Map data and usage

Map tiles are © their provider and © OpenStreetMap contributors. Downloads follow your provider's terms and API-key limits. Place search uses OpenStreetMap Nominatim, which allows no more than 1 request/second and sends an identifying User-Agent. The app never bulk-downloads from tile.openstreetmap.org.
