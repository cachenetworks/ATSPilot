#pragma once

#include <optional>
#include <string>

namespace atspilot {

// A keyboard shortcut using Windows virtual-key codes. Modifiers must match
// exactly, so "F9" does not fire while the game's own Ctrl+F9 is pressed.
struct KeyBinding {
    int virtualKey = 0;
    bool shift = false;
    bool ctrl = false;
    bool alt = false;

    bool operator==(const KeyBinding&) const = default;
};

// Parses e.g. "F9", "Shift+Delete", "Ctrl+Alt+K". Case-insensitive.
std::optional<KeyBinding> parseKeyBinding(const std::string& text);
std::string toString(const KeyBinding& k);

}  // namespace atspilot
