# Third-party notices

## Included in this repository or in builds

### SCS SDK 1.15 headers (`external/scs_sdk_1_15`)
Copyright (C) 2016 SCS Software. Distributed under the SCS SDK licence (MIT
terms); full text in `external/scs_sdk_1_15/LICENSE.txt`. Source:
https://modding.scssoft.com/wiki/Documentation/Engine/SDK/Telemetry

### ETS2LA game plugin structures (`third_party/ets2la_plugin`)
Copyright (c) 2024 Dario Wouters. MIT licence; full text in
`third_party/ets2la_plugin/LICENSE.md`. Game structure layouts and memory
patterns from https://github.com/ETS2LA/plugin (commit `3b01d90`), based on
https://github.com/dariowouters/ts-extra-utilities. ATSPilot's
`src/plugin/GameMemory.cpp` adapts its traffic and traffic-light collection.

### zlib 1.3.1 (fetched at build time, statically linked)
Copyright (C) 1995-2024 Jean-loup Gailly and Mark Adler. zlib licence:
https://zlib.net/zlib_license.html

### CityHash64 v1.0.3 (`src/core/map/CityHash.cpp`)
Copyright (c) 2011 Google, Inc. MIT licence. The algorithm is re-expressed for
the single 64-bit function that HashFS uses.

> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions: The above copyright
> notice and this permission notice shall be included in all copies or
> substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
> WITHOUT WARRANTY OF ANY KIND.

### doctest 2.4.11 (fetched at build time, tests only)
Copyright (c) 2016-2023 Viktor Kirilov. MIT licence.

## References (no code copied)

The following open-source projects were studied as **format references** for
HashFS archives, map sectors and prefab descriptors. ATSPilot's parsers are
independent implementations, and no source code from these projects is
included.

- truckermudgeon/maps, GPL-3.0: https://github.com/truckermudgeon/maps
- sk-zk/TruckLib and the map-docs wiki, GPL-2.0: https://github.com/sk-zk/TruckLib
- dariowouters/ts-map structure templates: https://github.com/dariowouters/ts-map
- SCS Software Blender Tools (prefab constants): https://github.com/SCSSoftware/BlenderTools

All format facts were then checked against the game data shipped with ATS 1.61
(see docs/map-parsing.md).
