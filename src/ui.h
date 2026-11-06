#pragma once
#include <Arduino.h>
#include <vector>

namespace ui {

constexpr uint16_t kBg = 0x0000;
constexpr uint16_t kFg = 0xFFFF;
constexpr uint16_t kDim = 0x8410;
constexpr uint16_t kAccent = 0x05FF;
constexpr uint16_t kBar = 0x18E3;
constexpr uint16_t kSel = 0x0339;
constexpr uint16_t kWarn = 0xFD20;
constexpr uint16_t kBad = 0xF800;
constexpr uint16_t kGood = 0x07E0;

void header(const char *title, const char *right = nullptr);
void footer(const char *hint);
void present();

// Scrollable list; returns selected index or -1 on back.
int menu(const char *title, const std::vector<String> &items, int initial = 0, const char *hint = nullptr);
// Single-line editor; returns false if cancelled.
bool textInput(const char *title, String &value, bool secret = false, const char *prompt = nullptr);
void message(const char *title, const String &body, bool wait = true);
bool confirm(const char *title, const String &body);
// Draws a wrapped paragraph; returns the y after the last line.
int paragraph(const String &text, int x, int y, int w, uint16_t color = kFg);

}  // namespace ui
