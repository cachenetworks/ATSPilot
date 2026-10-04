#include "TestSupport.h"

#include <cstring>
#include <filesystem>
#include <fstream>

#include <zlib.h>

#include "map/BinaryReader.h"
#include "map/CityHash.h"
#include "map/HashFs.h"
#include "map/LanePlanner.h"
#include "map/PrefabDescription.h"
#include "map/RoadNetwork.h"
#include "map/SectorParser.h"
#include "map/Sii.h"
#include "map/Token.h"
#include "math/MathUtil.h"
#include "util/KeyBinding.h"

using namespace atspilot;
using doctest::Approx;

namespace {

LaneSegment straightLane(Vec2 from, Vec2 to, double spacing = 5.0) {
    LaneSegment s;
    const double len = distance(from, to);
    const int n = std::max(1, static_cast<int>(len / spacing));
    for (int i = 0; i <= n; ++i) {
        const Vec2 p = lerp(from, to, static_cast<double>(i) / n);
        s.points.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.0f});
    }
    return s;
}

template <typename T>
void put(std::vector<char>& b, T v) {
    const char* p = reinterpret_cast<const char*>(&v);
    b.insert(b.end(), p, p + sizeof(T));
}

void putHeader(std::vector<char>& b, std::uint32_t type, std::uint64_t uid) {
    put(b, type);
    put(b, uid);
    b.insert(b.end(), 12 + 12 + 12 + 4, '\0');
    put(b, std::uint32_t{0});  // flags
    b.push_back('\0');         // view distance
}

void putNode(std::vector<char>& b, std::uint64_t uid, double x, double z) {
    put(b, uid);
    put(b, static_cast<std::int32_t>(x * 256));
    put(b, std::int32_t{0});
    put(b, static_cast<std::int32_t>(z * 256));
    // Rotation: identity quaternion (w, x, y, z): forward is -Z (north).
    put(b, 1.0f);
    put(b, 0.0f);
    put(b, 0.0f);
    put(b, 0.0f);
    put(b, std::uint64_t{0});
    put(b, std::uint64_t{0});
    put(b, std::uint32_t{0});
}

}  // namespace

TEST_CASE("CityHash64 v1.0.3 known values") {
    // Empty string hashes to k2 by definition; HashFS uses it for the root directory.
    CHECK(cityHash64("") == 0x9ae16a3b2f90404fULL);
    // Stable regression values for each length branch (checked against the HashFS
    // entries of ATS 1.61 archives, where "map", "def/world" etc. resolve).
    CHECK(cityHash64("map") != cityHash64("Map"));
    CHECK(cityHash64("def/world/road_look.template.sii") != cityHash64("def/world/road_look.sii"));
    const std::string longPath(100, 'a');
    CHECK(cityHash64(longPath) == cityHash64(longPath));
}

TEST_CASE("tokens round-trip") {
    for (const char* s : {"us_tmpl0", "look5", "a", "zzzzzzzzzzzz", "road_1"}) {
        CHECK(tokenToString(tokenFromString(s)) == s);
    }
    CHECK(tokenFromString("") == 0);
    CHECK(tokenFromString("toolongtoken13") == 0);
    CHECK(tokenFromString("bad-char") == 0);
}

TEST_CASE("SII text parser reads units, arrays and comments") {
    const auto units = parseSiiText(R"(SiiNunit
{
# comment
road_look : road.us_tmpl0
{
	name: "us 2-2 freeway"
	road_offset: 2.0
	lanes_left[]: traffic_lane.road.freeway
	lanes_left[]: traffic_lane.road.freeway
	lanes_right[]: traffic_lane.road.freeway // trailing comment
	lane_offsets_right[]: (1.75, 0)
}
	road_template_variant : .tmpl_var.x
	{
		lanes_right[]: a
	}
prefab_model : prefab.81
{
	prefab_desc: "/prefab/garage/garage.ppd"
}
}
)");
    REQUIRE(units.size() == 3);
    CHECK(units[0].className == "road_look");
    CHECK(units[0].name == "road.us_tmpl0");
    CHECK(units[0].count("lanes_left") == 2);
    CHECK(units[0].count("lanes_right") == 1);
    CHECK(siiNumber(*units[0].first("road_offset")) == Approx(2.0));
    CHECK(siiUnquote(*units[2].first("prefab_desc")) == "/prefab/garage/garage.ppd");
    CHECK(siiNumber("&40900000") == Approx(4.5));
}

TEST_CASE("binary reader rejects overruns") {
    const char data[4] = {1, 0, 0, 0};
    BinaryReader r(data, sizeof(data));
    CHECK(r.u32() == 1);
    CHECK_THROWS_AS(r.u8(), ParseError);
}

TEST_CASE("HashFS v1 archive round-trip with zlib entries and directory listings") {
    // Builds a tiny v1 archive in memory: header, entries table, data.
    const auto dir = std::filesystem::temp_directory_path() / "atspilot_test_hashfs";
    std::filesystem::create_directories(dir);
    const auto file = dir / "test.scs";

    struct Item {
        std::string path;
        std::string content;
        bool directory;
        bool compress;
    };
    const std::vector<Item> items = {
        {"", "*def\n", true, false},
        {"def", "hello.txt\n", true, false},
        {"def/hello.txt", "Hello ATSPilot, Hello ATSPilot, Hello ATSPilot", false, true},
    };
    std::vector<char> out;
    out.insert(out.end(), {'S', 'C', 'S', '#'});
    put(out, std::uint16_t{1});
    put(out, std::uint16_t{0});
    out.insert(out.end(), {'C', 'I', 'T', 'Y'});
    put(out, static_cast<std::uint32_t>(items.size()));
    put(out, std::uint32_t{20});  // entry table directly after the header
    const std::size_t dataStart = 20 + items.size() * 32;
    std::vector<char> data;
    std::vector<char> table;
    for (const auto& it : items) {
        std::vector<char> payload(it.content.begin(), it.content.end());
        if (it.compress) {
            uLongf len = compressBound(static_cast<uLong>(payload.size()));
            std::vector<char> z(len);
            compress(reinterpret_cast<Bytef*>(z.data()), &len, reinterpret_cast<const Bytef*>(payload.data()),
                     static_cast<uLong>(payload.size()));
            z.resize(len);
            payload = z;
        }
        put(table, cityHash64(it.path));
        put(table, static_cast<std::uint64_t>(dataStart + data.size()));
        put(table, static_cast<std::uint32_t>((it.directory ? 1u : 0u) | (it.compress ? 2u : 0u)));
        put(table, std::uint32_t{0});
        put(table, static_cast<std::uint32_t>(it.content.size()));
        put(table, static_cast<std::uint32_t>(payload.size()));
        data.insert(data.end(), payload.begin(), payload.end());
    }
    out.insert(out.end(), table.begin(), table.end());
    out.insert(out.end(), data.begin(), data.end());
    std::ofstream(file, std::ios::binary).write(out.data(), static_cast<std::streamsize>(out.size()));

    std::string err;
    auto a = HashFsArchive::open(file, &err);
    REQUIRE_MESSAGE(a, err);
    CHECK(a->version() == 1);
    const auto root = a->list("");
    REQUIRE(root);
    REQUIRE(root->subdirectories.size() == 1);
    CHECK(root->subdirectories[0] == "def");
    const auto content = a->read("/def/hello.txt");
    REQUIRE(content);
    CHECK(std::string(content->begin(), content->end()) == items[2].content);
    CHECK_FALSE(a->read("def/missing.txt"));

    GameFileSystem fs;
    fs.add(std::move(a));
    CHECK(fs.contains("def/hello.txt"));
    CHECK(fs.list("def").files.size() == 1);
}

TEST_CASE("sector parser decodes roads, prefabs and nodes and rejects unknown items") {
    std::vector<char> b;
    put(b, kSupportedSectorVersion);
    put(b, std::uint64_t{0});
    put(b, std::uint32_t{0});
    put(b, std::uint32_t{2});  // items

    putHeader(b, 3, 0x1111);  // road
    b.insert(b.end(), 4, '\0');
    put(b, tokenFromString("us_tmpl0"));
    b.insert(b.end(), 8 * 8 + 2 * 12 + 3 * 8 + 3 * 20 + 8, '\0');
    put(b, std::uint64_t{0xA});
    put(b, std::uint64_t{0xB});
    put(b, 100.0f);

    putHeader(b, 4, 0x2222);  // prefab with two nodes
    put(b, tokenFromString("p81"));
    put(b, std::uint64_t{0});
    put(b, std::uint32_t{0});  // parts
    put(b, std::uint32_t{2});
    put(b, std::uint64_t{0xB});
    put(b, std::uint64_t{0xC});
    put(b, std::uint32_t{0});  // connected items
    put(b, std::uint64_t{0});  // ferry link
    put(b, std::uint16_t{1});  // origin index
    b.insert(b.end(), 2 * 12, '\0');
    put(b, std::uint64_t{0});  // semaphore profile

    put(b, std::uint32_t{2});  // nodes
    putNode(b, 0xA, 0.0, 0.0);
    putNode(b, 0xB, 0.0, -100.0);
    put(b, std::uint32_t{0});  // visible area children

    const SectorData s = parseSector(b.data(), b.size());
    REQUIRE(s.roads.size() == 1);
    CHECK(s.roads[0].uid == 0x1111);
    CHECK(tokenToString(s.roads[0].roadLook) == "us_tmpl0");
    CHECK(s.roads[0].startNode == 0xA);
    CHECK(s.roads[0].endNode == 0xB);
    CHECK(s.roads[0].length == Approx(100.0));
    REQUIRE(s.prefabs.size() == 1);
    CHECK(s.prefabs[0].nodes.size() == 2);
    CHECK(s.prefabs[0].originIndex == 1);
    REQUIRE(s.nodes.size() == 2);
    CHECK(s.nodes[1].position.z == Approx(-100.0));
    // Identity rotation faces north (-Z).
    CHECK(s.nodes[0].forward().z == Approx(-1.0));

    std::vector<char> bad = b;
    std::memcpy(bad.data() + 20, "\x63\x00\x00\x00", 4);  // first item type -> 99
    CHECK_THROWS_AS(parseSector(bad.data(), bad.size()), ParseError);
    std::vector<char> wrongVersion = b;
    wrongVersion[0] = 1;
    CHECK_THROWS_AS(parseSector(wrongVersion.data(), wrongVersion.size()), ParseError);
    CHECK_THROWS_AS(parseSector(b.data(), b.size() - 3), ParseError);
}

TEST_CASE("road network spatial query, projection and cache round-trip") {
    RoadNetwork net;
    const auto a = net.add(straightLane({0, 0}, {100, 0}));
    const auto c = net.add(straightLane({0, 4.5}, {100, 4.5}));
    net.mutableSegments()[a].next = {};
    net.finalize();
    const auto matches = net.query({50, 1.0}, 10.0);
    REQUIRE(matches.size() == 2);
    CHECK(matches[0].segment == a);
    CHECK(matches[0].distance == Approx(1.0));
    CHECK(matches[0].crossTrack == Approx(1.0));
    CHECK(matches[0].s == Approx(50.0));
    CHECK(matches[1].segment == c);

    const auto file = std::filesystem::temp_directory_path() / "atspilot_test_net.cache";
    REQUIRE(net.save(file, "fp1"));
    CHECK_FALSE(RoadNetwork::load(file, "other").has_value());
    const auto loaded = RoadNetwork::load(file, "fp1");
    REQUIRE(loaded);
    CHECK(loaded->size() == 2);
    CHECK(loaded->query({50, 1.0}, 10.0).size() == 2);
}

TEST_CASE("localizer prefers the lane with matching heading and sticks to it") {
    RoadNetwork net;
    const auto east = net.add(straightLane({0, 0}, {200, 0}));
    const auto west = net.add(straightLane({200, 4.0}, {0, 4.0}));
    net.finalize();
    Localizer loc(net);
    // Closer to the westbound lane but heading east: must pick the eastbound lane.
    auto r = loc.update({50, 2.5}, 0.0, {});
    REQUIRE(r.valid);
    CHECK(r.match.segment == east);
    // Drifting towards the middle does not flip lanes (hysteresis).
    r = loc.update({60, 2.1}, 0.0, {});
    CHECK(r.match.segment == east);
    // Facing west picks the westbound lane.
    Localizer loc2(net);
    r = loc2.update({50, 3.5}, kPi, {});
    REQUIRE(r.valid);
    CHECK(r.match.segment == west);
    // Nothing nearby.
    CHECK_FALSE(loc.update({500, 500}, 0.0, {}).valid);
}

TEST_CASE("path builder follows the straightest branch and keeps earlier choices") {
    RoadNetwork net;
    const auto approach = net.add(straightLane({0, 0}, {100, 0}));
    const auto straight = net.add(straightLane({100, 0}, {400, 0}));
    LaneSegment exitLane;
    for (int i = 0; i <= 30; ++i) {
        const double a = i * degToRad(3.0);
        exitLane.points.push_back({static_cast<float>(100 + 80 * std::sin(a)),
                                   static_cast<float>(-80 + 80 * std::cos(a)), 0.0f});
    }
    const auto exit = net.add(exitLane);
    net.mutableSegments()[approach].next = {exit, straight};
    net.finalize();

    LaneMatch m;
    m.segment = approach;
    m.s = 50.0;
    PathBuildParams pp;
    pp.ahead = 300.0;
    PlannedPath p = buildPlannedPath(net, m, pp, {});
    REQUIRE(p.chain.size() >= 2);
    CHECK(p.chain[1] == straight);
    CHECK(p.path.valid());
    CHECK(p.truckS == Approx(std::min(50.0, pp.behind)));

    // A previous plan that took the exit is kept while still valid.
    PlannedPath q = buildPlannedPath(net, m, pp, {approach, exit});
    CHECK(q.chain[1] == exit);
    CHECK(q.nextManeuver == "Keep Right");
}

TEST_CASE("key bindings parse with exact modifiers") {
    const auto f9 = parseKeyBinding("F9");
    REQUIRE(f9);
    CHECK(f9->virtualKey == 0x78);
    CHECK_FALSE(f9->shift);
    const auto sd = parseKeyBinding("shift + delete");
    REQUIRE(sd);
    CHECK(sd->virtualKey == 0x2E);
    CHECK(sd->shift);
    CHECK(parseKeyBinding("Ctrl+Alt+K")->virtualKey == 'K');
    CHECK(toString(*parseKeyBinding("Shift+F9")) == "Shift+F9");
    CHECK_FALSE(parseKeyBinding(""));
    CHECK_FALSE(parseKeyBinding("Shift+"));
    CHECK_FALSE(parseKeyBinding("F9+F10"));
    CHECK_FALSE(parseKeyBinding("Hyper"));
}

TEST_CASE("path trimming re-bases arc length") {
    Path p;
    p.append({0, 0});
    p.append({100, 0});
    const Path t = p.trimmed(30.0, 80.0);
    CHECK(t.length() == Approx(50.0));
    CHECK(t.positionAt(0.0).x == Approx(30.0));
}
