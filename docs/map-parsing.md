# Map parsing

ATSPilot builds a lane-level road graph from the same files the game loads.
Everything here was checked against ATS 1.61 data. `atspilot_mapdump build`
reproduces the numbers below.

## Archives (HashFS)

The `.scs` files in the game folder are HashFS archives (`SCS#` magic,
`CITY` hash method). ATS 1.61 ships **version 2**. Version 1 is also supported
(mods).

v2 header (little-endian):

| Offset | Field |
|---|---|
| 0 | magic `SCS#` |
| 4 | u16 version (2), u16 salt |
| 8 | `CITY` |
| 12 | u32 entry count |
| 16 | u32 entry table compressed size |
| 20 | u32 metadata table size **in 32-bit words** |
| 24 | u32 metadata table compressed size |
| 28 | u64 entry table offset |
| 36 | u64 metadata table offset |

**Finding:** the metadata size field counts 4-byte words, not bytes (base_map:
43,630 words = 174,520 bytes after inflation). Both tables are zlib streams.

Entries are 16 bytes: `u64 hash, u32 metadata index, u16 metadata count, u8
flags, u8`. Each metadata header word is `u24 index | u8 type`. A plain (0x80)
or directory (0x81) payload is 16 bytes: `u32 (compressed size : 28,
compression : 4), u32 size : 28, u32, u32 offset/16`. Compression 1 is zlib.
GDeflate (3) only appears on texture data and is not needed.

v2 directory listings are `u32 count`, `count` length bytes, then the names, with
subdirectories prefixed by `/`.

**Finding:** path hashes are **CityHash64 v1.0.3**. v1.1 produces different
values for short strings. Paths are hashed without a leading slash, and the root
is `""`.

## Definitions

- `def/world/road_look*.sii` (plain-text SII): `road_look : road.<token>` units
  with `lanes_left[]`, `lanes_right[]`, `road_offset`, `lane_offsets_*[]`.
  ATS 1.61 has 212 looks (34 classic plus 178 templates).
- `def/world/prefab*.sii`: `prefab_model : prefab.<token>` with `prefab_desc`
  (path to the `.ppd`).

**Finding:** templated road looks do not state a lane width. See calibration
below.

## Sectors (`map/usa/sec±XXXX±YYYY.base`, version 907)

`u32 version, u64 game id, u32 padding, u32 item count, items…, u32 node count,
nodes…, u32 count + u64[]`. Items have **no size prefix**, so every item type
has to be decoded to reach the next one. `SectorParser.cpp` implements all 32
types seen in 1.61, using the layouts documented by the projects listed in
THIRD_PARTY_NOTICES.md. An unknown type, an overrun or trailing bytes fails that
sector with a logged error instead of producing wrong geometry.

- Item header: `u32 type, u64 uid, 3×float3 + float, u32 flags, u8 view distance`.
- Road (type 3): road look token, start/end node uids, length.
- Prefab (type 4): model token, node uids, origin index.
- Node (56 bytes): `u64 uid, i32×3 position (÷256 m), float4 rotation (w,x,y,z),
  u64 backward item, u64 forward item, u32 flags`. Forward = rotation applied to (0,0,−1).

Result on 1.61: **925/925 sectors, 0 errors, 224,870 roads, 84,855 prefabs.**

## Prefabs (`.ppd`, version 25)

Header counts are followed by u32 table offsets. ATSPilot reads the nodes (104
bytes: position, direction, input/output lane curve indices) and nav curves (132
bytes: start/end position and rotation, next/previous curve indices).

A prefab is placed by aligning its descriptor node `originIndex` with the map
node `nodes[0]`: rotate by `angle(map node forward) − angle(ppd node direction)`
in the ground plane, then translate. All 1,737 descriptors used by the map parse.

## Lane graph

1. **Junction curves:** each prefab nav curve becomes a lane segment (a Hermite
   spline whose tangent length equals the chord), linked by the descriptor's
   `next` indices.
2. **Road centre lines:** a Hermite spline between the two node positions and
   orientations. Orientations pointing against the road are flipped, and sample
   density follows curvature.
3. **Lane offsets (calibrated):** for every road look, ATSPilot measures where
   the junction lanes start and end relative to the road's centre line, at
   every road end that touches a prefab. A road end counts only when exactly
   one run of `laneCount` aligned junction lanes with lane-like spacing
   (2.5–6 m) exists, so an extra ramp lane makes it ambiguous and it is
   skipped. The per-lane median gives the offsets. 267 look sides are
   calibrated. Uncalibrated looks fall back to
   `road_offset/2 + (i + ½)·lane_width`, or to centred lanes for one-way looks
   (default lane width 4.5 m).

   **Finding:** for one-way carriageways the node line is **not** the inner
   edge. A 3-lane "us 0-3 freeway" has lane centres at −6.75, −2.25 and +2.25 m
   relative to its nodes. Offsets therefore have to be measured on both sides
   of the node line. The first calibration ignored negative offsets, mis-placed
   such roads, and fragmented the graph (24,227 missed links before the fix,
   2,564 after).
4. **Connections:** a lane end links to any lane start within 1.6 m and 30° of
   heading. This is purely geometric, so it does not depend on undocumented
   lane-index conventions.
5. **Gaps and merges:** a lane end with no successor links either to an
   aligned start straight ahead (lateral ≤ 1.6 m, up to 25 m ahead), for
   junction curves that begin past the node, or to the nearest aligned start
   within 5 m (a merge or lane drop). The path builder turns any lateral gap
   at a join into a smooth 60 m transition.
6. **Snapping:** for regular joins only (≤ 1.6 m), the remaining offset
   between a road lane end and its junction curve is spread along the road
   lane. Blend factors are taken from the unmodified geometry. An earlier
   version measured them on already-shifted points, which made geometry on
   very short roads grow exponentially. The builder now also drops any
   segment with non-finite or out-of-map coordinates (0 on 1.61).
7. **Lane changes** are not stored as edges. Parallel lanes of the same road
   and direction are found on demand (`laneNeighbors`) by the route planner.

Diagnostics on 1.61 (`atspilot_mapdump build`):

- 1,537,559 lane segments; **97.2%** of lane ends connected; 973 merge and
  758 gap links.
- Road–junction joins: 71% within 0.25 m. Larger gaps are **longitudinal**
  (the junction curve ends 1.1–1.5 m before the road lane starts), with
  lateral error ≈ 0.
- 20,523 road dead ends are genuine: the map node has nothing attached, as
  with stubs or closed areas (verified with `atspilot_mapdump item`). 2,564
  remain where the map continues but no lane link was found.
- 230,857 prefab node placements checked: 1 off by more than 1 m.

## Destinations and routing

Company items (type 6: city token, prefab uid, company token, node uid) give
2,186 depots on 1.61. Their tokens match the job configuration's
`destination.city.id` and `destination.company.id`.

**Finding:** most depot prefabs have no nav curves of their own. A depot is
therefore reached through the lanes that end at its prefab nodes (the
entrance). 2,106 depots resolve this way.

`planRoute` runs A* over lane successors (cost: length) and lane changes
(cost: 60 m by default) to any destination lane. Its straight-line heuristic,
measured to the destination area, never overestimates. Examples on 1.61:
Sacramento → Los Angeles 41.1 km in 12 ms; Sacramento → Houston 173.7 km in
130 ms (365k expansions). Routing runs asynchronously on the planner thread,
and is recalculated once the truck has been off the route (neither on it nor
beside it) for 1 s.

## Cache

The graph is saved to `cache/map_usa.cache` (binary, format version 1)
together with a fingerprint of every archive's name, size and timestamp and the
lane width. A game update, DLC change or new ATSPilot cache format forces a
rebuild (about 15 s on an i7-9700K) on the background thread.

## Known gaps

- Speed limits per road are not extracted. ATSPilot uses
  `truck.navigation.speed.limit`.
- Lane drops and merges without a geometric successor end the path, so the
  planner slows to a stop before the end. Merge handling is on the roadmap.
- Memory: about 11 M lane points (≈130 MB) for the full USA map. Region-based
  loading is a planned optimisation.
- `.zip` mods are not read.
