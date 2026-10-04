#include "Hud.h"

#include <windows.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "math/MathUtil.h"

namespace atspilot::plugin {
namespace {

constexpr wchar_t kClassName[] = L"ATSPilotHud";
constexpr UINT_PTR kTimer = 1;

struct FindGame {
    DWORD pid;
    HWND self;
    HWND best = nullptr;
    LONG bestArea = 0;
};

BOOL CALLBACK enumWindow(HWND hwnd, LPARAM lp) {
    auto* f = reinterpret_cast<FindGame*>(lp);
    DWORD pid = 0;
    GetWindowThreadProcessId(hwnd, &pid);
    if (pid != f->pid || hwnd == f->self || !IsWindowVisible(hwnd)) return TRUE;
    RECT r;
    GetClientRect(hwnd, &r);
    const LONG area = (r.right - r.left) * (r.bottom - r.top);
    if (area > f->bestArea) {
        f->bestArea = area;
        f->best = hwnd;
    }
    return TRUE;
}

HWND findGameWindow(HWND self) {
    FindGame f{GetCurrentProcessId(), self};
    EnumWindows(enumWindow, reinterpret_cast<LPARAM>(&f));
    return f.best;
}

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

std::wstring speedText(double mps, SpeedUnits u) {
    return std::to_wstring(std::lround(mpsToSpeed(mps, u))) + (u == SpeedUnits::Mph ? L" mph" : L" km/h");
}

std::wstring distanceText(double metres, SpeedUnits u) {
    wchar_t buf[32];
    if (u == SpeedUnits::Mph) {
        const double miles = metres / 1609.344;
        if (miles < 0.2) swprintf(buf, 32, L"%.0f ft", metres * 3.28084);
        else swprintf(buf, 32, L"%.1f mi", miles);
    } else {
        if (metres < 1000.0) swprintf(buf, 32, L"%.0f m", metres);
        else swprintf(buf, 32, L"%.1f km", metres / 1000.0);
    }
    return buf;
}

struct Row {
    std::wstring label;
    std::wstring value;
    COLORREF color;
};

}  // namespace

Hud::Hud(const HudConfig& cfg, SpeedUnits units) : cfg_(cfg), units_(units) {
    thread_ = std::thread([this] { run(); });
}

Hud::~Hud() {
    stop_ = true;
    if (HWND w = static_cast<HWND>(window_.load())) PostMessageW(w, WM_CLOSE, 0, 0);
    if (thread_.joinable()) thread_.join();
}

void Hud::update(const PilotStatus& status) {
    std::lock_guard lock(mutex_);
    status_ = status;
}

void Hud::run() {
    HINSTANCE inst = GetModuleHandleW(nullptr);
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = DefWindowProcW;
    wc.hInstance = inst;
    wc.lpszClassName = kClassName;
    RegisterClassExW(&wc);  // fails harmlessly if already registered by an earlier load

    // Click-through, never activated, not in the task bar.
    HWND hwnd = CreateWindowExW(WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST | WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE,
                                kClassName, L"ATSPilot", WS_POPUP, 0, 0, 10, 10, nullptr, nullptr, inst, nullptr);
    if (!hwnd) return;
    window_ = hwnd;
    SetTimer(hwnd, kTimer, 250, nullptr);

    const bool standalone = GetEnvironmentVariableW(L"ATSPILOT_HUD_STANDALONE", nullptr, 0) > 0;
    const double scale = cfg_.scale;
    const int width = static_cast<int>(300 * scale);
    const int rowH = static_cast<int>(22 * scale);
    const int pad = static_cast<int>(12 * scale);
    HFONT titleFont = CreateFontW(static_cast<int>(-18 * scale), 0, 0, 0, FW_BOLD, 0, 0, 0, DEFAULT_CHARSET, 0, 0,
                                  CLEARTYPE_QUALITY, 0, L"Segoe UI");
    HFONT bodyFont = CreateFontW(static_cast<int>(-15 * scale), 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET, 0, 0,
                                 CLEARTYPE_QUALITY, 0, L"Segoe UI");
    const BYTE bgAlpha = static_cast<BYTE>(clamp(cfg_.opacity, 0.2, 1.0) * 200.0);
    const COLORREF bg = RGB(18, 20, 24);

    MSG msg;
    while (!stop_ && GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (msg.message == WM_CLOSE && msg.hwnd == hwnd) break;
        if (msg.message != WM_TIMER) {
            DispatchMessageW(&msg);
            continue;
        }

        HWND game = findGameWindow(hwnd);
        const bool visible = standalone || (game && GetForegroundWindow() == game && !IsIconic(game));
        if (!visible) {
            ShowWindow(hwnd, SW_HIDE);
            continue;
        }

        PilotStatus st;
        {
            std::lock_guard lock(mutex_);
            st = status_;
        }
        const bool on = st.mode != PilotMode::Off;
        std::vector<Row> rows;
        const COLORREF white = RGB(235, 238, 242), grey = RGB(150, 156, 166), green = RGB(80, 220, 120),
                       amber = RGB(255, 190, 60), red = RGB(255, 90, 80);
        rows.push_back({toString(st.mode) == std::string("OFF") ? L"ATSPilot" : widen(toString(st.mode)),
                        on ? L"ACTIVE" : (st.available ? L"READY" : L"UNAVAILABLE"),
                        st.mode == PilotMode::EmergencyStop ? red : on ? green : st.available ? white : grey});
        rows.push_back({L"Set speed", speedText(st.setSpeed, units_), white});
        if (on) rows.push_back({L"Target", speedText(st.targetSpeed, units_), white});
        rows.push_back({L"Cruise control", st.gameCruiseActive ? L"ON  " + speedText(st.cruiseSetSpeed, units_) : L"off",
                        st.gameCruiseActive ? green : grey});
        rows.push_back({L"Navigation",
                        st.navigationActive ? (st.gpsMatched ? L"Active (GPS)" : L"Active") : L"Follow road",
                        st.navigationActive ? green : white});
        if (!st.nextManeuver.empty()) {
            rows.push_back({L"Next", widen(st.nextManeuver) + L"  " + distanceText(st.nextManeuverDistance, units_), white});
        }
        if (st.navigationActive) rows.push_back({L"Remaining", distanceText(st.routeDistance, units_), white});
        if (st.trafficAware) {
            rows.push_back({L"Traffic", st.leadDistance >= 0.0 ? distanceText(st.leadDistance, units_) + L" ahead"
                                                               : std::wstring(L"clear"),
                            st.leadDistance >= 0.0 && st.leadDistance < 30.0 ? amber : green});
        }
        if (!st.signalState.empty()) {
            const bool go = st.signalState == "green";
            rows.push_back({L"Light", widen(st.signalState), go ? green : st.signalState == "red" ? red : amber});
        }
        if (!st.profile.empty()) rows.push_back({L"Profile", widen(st.profile), grey});

        const int height = pad * 2 + rowH * static_cast<int>(rows.size() + 1);

        // Position over the game window's client area (or the primary monitor).
        RECT area{0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN)};
        if (game) {
            RECT c;
            GetClientRect(game, &c);
            POINT tl{c.left, c.top}, br{c.right, c.bottom};
            ClientToScreen(game, &tl);
            ClientToScreen(game, &br);
            area = {tl.x, tl.y, br.x, br.y};
        }
        const int margin = static_cast<int>(24 * scale);
        const bool right = cfg_.corner.find("right") != std::string::npos;
        const bool bottom = cfg_.corner.find("bottom") != std::string::npos;
        const POINT pos{right ? area.right - width - margin : area.left + margin,
                        bottom ? area.bottom - height - margin : area.top + margin};

        // Draw with GDI into a 32-bit DIB, then derive per-pixel alpha: background
        // pixels get the panel opacity, anything drawn on top is opaque.
        HDC screen = GetDC(nullptr);
        HDC dc = CreateCompatibleDC(screen);
        BITMAPINFO bi{};
        bi.bmiHeader.biSize = sizeof(bi.bmiHeader);
        bi.bmiHeader.biWidth = width;
        bi.bmiHeader.biHeight = -height;
        bi.bmiHeader.biPlanes = 1;
        bi.bmiHeader.biBitCount = 32;
        void* bits = nullptr;
        HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
        HGDIOBJ oldBmp = SelectObject(dc, bmp);
        RECT full{0, 0, width, height};
        HBRUSH brush = CreateSolidBrush(bg);
        FillRect(dc, &full, brush);
        DeleteObject(brush);
        SetBkMode(dc, TRANSPARENT);

        int y = pad;
        HGDIOBJ oldFont = SelectObject(dc, titleFont);
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const Row& r = rows[i];
            SelectObject(dc, i == 0 ? titleFont : bodyFont);
            RECT lr{pad, y, width - pad, y + rowH};
            SetTextColor(dc, i == 0 ? r.color : grey);
            DrawTextW(dc, r.label.c_str(), -1, &lr, DT_LEFT | DT_VCENTER | DT_SINGLELINE);
            SetTextColor(dc, r.color);
            DrawTextW(dc, r.value.c_str(), -1, &lr, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
            y += i == 0 ? rowH + static_cast<int>(6 * scale) : rowH;
        }
        // Status message across the bottom.
        SelectObject(dc, bodyFont);
        RECT mr{pad, y, width - pad, y + rowH};
        const bool warn = st.statusMessage.find("Override") != std::string::npos ||
                          st.statusMessage.find("unavailable") != std::string::npos ||
                          st.statusMessage.find("Emergency") != std::string::npos ||
                          st.statusMessage.find("take over") != std::string::npos;
        SetTextColor(dc, warn ? amber : white);
        DrawTextW(dc, widen(st.statusMessage).c_str(), -1, &mr, DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
        SelectObject(dc, oldFont);
        GdiFlush();

        auto* px = static_cast<std::uint32_t*>(bits);
        // DIB pixels are BGRA in memory, i.e. 0x00RRGGBB as a little-endian word.
        const std::uint32_t bgPixel = (static_cast<std::uint32_t>(bg & 0xFF) << 16) | (bg & 0xFF00) | ((bg >> 16) & 0xFF);
        for (int i = 0; i < width * height; ++i) {
            const std::uint32_t rgb = px[i] & 0x00FFFFFFu;
            const std::uint32_t a = rgb == bgPixel ? bgAlpha : 255u;
            const std::uint32_t r = ((rgb >> 16) & 0xFF) * a / 255, g = ((rgb >> 8) & 0xFF) * a / 255, b = (rgb & 0xFF) * a / 255;
            px[i] = (a << 24) | (r << 16) | (g << 8) | b;  // premultiplied BGRA
        }

        SIZE size{width, height};
        POINT src{0, 0};
        BLENDFUNCTION blend{};
        blend.BlendOp = AC_SRC_OVER;
        blend.SourceConstantAlpha = 0xFF;
        blend.AlphaFormat = AC_SRC_ALPHA;
        UpdateLayeredWindow(hwnd, screen, const_cast<POINT*>(&pos), &size, dc, &src, 0, &blend, ULW_ALPHA);
        ShowWindow(hwnd, SW_SHOWNOACTIVATE);
        SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);

        SelectObject(dc, oldBmp);
        DeleteObject(bmp);
        DeleteDC(dc);
        ReleaseDC(nullptr, screen);
    }

    KillTimer(hwnd, kTimer);
    DestroyWindow(hwnd);
    window_ = nullptr;
    DeleteObject(titleFont);
    DeleteObject(bodyFont);
}

}  // namespace atspilot::plugin
