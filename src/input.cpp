#include "input.h"

#include <M5Cardputer.h>

Key readKey(bool textMode) {
    static uint32_t pressedAt = 0;
    static uint32_t lastRepeat = 0;
    static bool held = false;
    constexpr uint32_t kRepeatDelay = 450;
    constexpr uint32_t kRepeatEvery = 110;

    M5Cardputer.update();
    Key k;
    if (M5Cardputer.BtnA.wasPressed()) {
        k.any = k.enter = true;
        return k;
    }

    auto &kb = M5Cardputer.Keyboard;
    const bool changed = kb.isChange();
    if (!kb.isPressed()) {
        held = false;
        return k;
    }
    const uint32_t now = millis();
    if (changed) {
        pressedAt = lastRepeat = now;
        held = true;
    } else {
        if (!held || now - pressedAt < kRepeatDelay || now - lastRepeat < kRepeatEvery) return k;
        lastRepeat = now;
    }

    const auto &s = kb.keysState();
    k.fn = s.fn;
    k.enter = s.enter;
    k.del = s.del;  // backspace key
    if (!s.word.empty()) k.ch = s.word.back();
    if (s.space && !k.ch) k.ch = ' ';

    // Arrows are printed on ; , . / and escape on ` - take them with fn always,
    // and without fn outside of text fields.
    if (s.fn || !textMode) {
        bool navKey = true;
        switch (k.ch) {
            case ';': k.up = true; break;
            case '.': k.down = true; break;
            case ',': k.left = true; break;
            case '/': k.right = true; break;
            case '`': k.back = true; break;
            default: navKey = false; break;
        }
        if (navKey) k.ch = 0;
    }
    k.any = k.ch || k.enter || k.del || k.up || k.down || k.left || k.right || k.back;
    return k;
}
