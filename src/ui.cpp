#include "ui.h"

#include "app.h"
#include "input.h"

using app::canvas;

namespace ui {

void header(const char *title, const char *right) {
    canvas.fillRect(0, 0, app::kW, 14, kBar);
    canvas.setFont(&fonts::Font0);
    canvas.setTextSize(1);
    canvas.setTextColor(kFg, kBar);
    canvas.setTextDatum(middle_left);
    canvas.drawString(title, 4, 7);
    if (right) {
        canvas.setTextDatum(middle_right);
        canvas.drawString(right, app::kW - 4, 7);
    }
    canvas.setTextDatum(top_left);
}

void footer(const char *hint) {
    canvas.fillRect(0, app::kH - 11, app::kW, 11, kBar);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(kDim, kBar);
    canvas.setTextDatum(middle_left);
    canvas.drawString(hint, 3, app::kH - 5);
    canvas.setTextDatum(top_left);
}

void present() { canvas.pushSprite(&M5Cardputer.Display, 0, 0); }

int paragraph(const String &text, int x, int y, int w, uint16_t color) {
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(color);
    const int maxChars = w / 6;
    int start = 0;
    const int len = text.length();
    while (start < len) {
        int nl = text.indexOf('\n', start);
        if (nl < 0) nl = len;
        int end = min(nl, start + maxChars);
        if (end < nl) {
            int sp = text.lastIndexOf(' ', end);
            if (sp > start) end = sp;
        }
        canvas.drawString(text.substring(start, end), x, y);
        y += 10;
        start = end;
        while (start < len && (text[start] == ' ')) ++start;
        if (start < len && text[start] == '\n') ++start;
    }
    return y;
}

int menu(const char *title, const std::vector<String> &items, int initial, const char *hint) {
    if (items.empty()) return -1;
    int sel = constrain(initial, 0, int(items.size()) - 1);
    constexpr int kRowH = 12;
    constexpr int kTop = 16;
    const int rows = (app::kH - kTop - 12) / kRowH;
    int scroll = 0;
    bool dirty = true;
    char pos[16];
    while (true) {
        app::service();
        Key k = readKey();
        if (k.up) { sel = (sel - 1 + items.size()) % items.size(); dirty = true; }
        if (k.down) { sel = (sel + 1) % items.size(); dirty = true; }
        if (k.left) { sel = max(0, sel - rows); dirty = true; }
        if (k.right) { sel = min(int(items.size()) - 1, sel + rows); dirty = true; }
        if (k.enter) return sel;
        if (k.back || k.del) return -1;
        if (!dirty) { delay(10); continue; }
        dirty = false;
        if (sel < scroll) scroll = sel;
        if (sel >= scroll + rows) scroll = sel - rows + 1;
        canvas.fillScreen(kBg);
        snprintf(pos, sizeof pos, "%d/%d", sel + 1, int(items.size()));
        header(title, pos);
        canvas.setFont(&fonts::Font0);
        for (int i = 0; i < rows && scroll + i < int(items.size()); ++i) {
            const int idx = scroll + i;
            const int y = kTop + i * kRowH;
            if (idx == sel) canvas.fillRoundRect(2, y, app::kW - 4, kRowH - 1, 2, kSel);
            canvas.setTextColor(kFg, idx == sel ? kSel : kBg);
            String s = items[idx];
            if (s.length() > 38) s = s.substring(0, 37) + "~";
            canvas.drawString(s, 6, y + 2);
        }
        footer(hint ? hint : ";/. move  Enter select  ` back");
        present();
    }
}

bool textInput(const char *title, String &value, bool secret, const char *prompt) {
    String buf = value;
    bool dirty = true;
    uint32_t blinkAt = millis();
    bool cursor = true;
    while (true) {
        app::service();
        Key k = readKey(true);
        if (k.any) {
            dirty = true;
            if (k.enter) { value = buf; return true; }
            if (k.back) return false;
            if (k.del && buf.length()) buf.remove(buf.length() - 1);
            else if (k.ch && buf.length() < 120) buf += k.ch;
        }
        if (millis() - blinkAt > 500) { blinkAt = millis(); cursor = !cursor; dirty = true; }
        if (!dirty) { delay(10); continue; }
        dirty = false;
        canvas.fillScreen(kBg);
        header(title);
        int y = 20;
        if (prompt) y = paragraph(prompt, 4, y, app::kW - 8, kDim) + 4;
        String shown = secret ? String() : buf;
        if (secret) for (size_t i = 0; i < buf.length(); ++i) shown += (i + 1 == buf.length()) ? buf[i] : '*';
        if (cursor) shown += '_';
        canvas.drawRect(2, y - 2, app::kW - 4, 50, kAccent);
        paragraph(shown, 6, y + 2, app::kW - 12);
        footer("Enter OK  fn+` cancel  Bksp delete");
        present();
    }
}

void message(const char *title, const String &body, bool wait) {
    canvas.fillScreen(kBg);
    header(title);
    paragraph(body, 4, 20, app::kW - 8);
    footer(wait ? "Press any key" : "");
    present();
    if (!wait) return;
    while (true) {
        app::service();
        if (readKey().any) return;
        delay(10);
    }
}

bool confirm(const char *title, const String &body) {
    canvas.fillScreen(kBg);
    header(title);
    paragraph(body, 4, 20, app::kW - 8);
    footer("Enter / y = yes    ` / n = no");
    present();
    while (true) {
        app::service();
        Key k = readKey();
        if (k.enter || k.ch == 'y' || k.ch == 'Y') return true;
        if (k.back || k.del || k.ch == 'n' || k.ch == 'N') return false;
        delay(10);
    }
}

}  // namespace ui
