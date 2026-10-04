# ets2la_plugin (vendored)

Game structure definitions and memory pattern scanning from
[ETS2LA/plugin](https://github.com/ETS2LA/plugin) at commit `3b01d90`
(2026-09-24, game 1.61), itself based on
[dariowouters/ts-extra-utilities](https://github.com/dariowouters/ts-extra-utilities).
MIT licensed, see `LICENSE.md`.

Copied unchanged: `src/prism`, `src/memory/memory_scan.hpp`,
`src/memory/memory_utils.*`, `src/patterns.hpp`, `src/patterns.win32.hpp`.
`shim/core.hpp` is ATSPilot's own replacement for the plugin's `core.hpp`, used
only for logging. The plugin's shared-memory transport and processing code are
not used; ATSPilot's `src/plugin/GameMemory.cpp` reads the structures directly.
