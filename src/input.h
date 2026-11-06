#pragma once

struct Key {
    bool any = false;
    char ch = 0;  // printable character, 0 if none
    bool enter = false;
    bool back = false;  // `/esc in menus, fn+` in text fields
    bool del = false;   // backspace
    bool up = false, down = false, left = false, right = false;
    bool fn = false;
};

// Polls the keyboard and G0 button. Returns a key once per press, with auto-repeat while held.
// In text mode ; , . / ` are delivered as characters; use fn+key for arrows / escape.
Key readKey(bool textMode = false);
