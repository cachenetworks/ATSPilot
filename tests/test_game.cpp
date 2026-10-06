#include <doctest/doctest.h>

#include "game/Game.h"

using namespace atspilot;

TEST_CASE("SCS game ids and selectors resolve to the correct game") {
    CHECK(gameKindFromId("ats") == GameKind::Ats);
    CHECK(gameKindFromId("eut2") == GameKind::Ets2);
    CHECK(gameKindFromId("unknown") == GameKind::Unknown);

    CHECK(gameKindFromSelector("ATS") == GameKind::Ats);
    CHECK(gameKindFromSelector("amtrucks.exe") == GameKind::Ats);
    CHECK(gameKindFromSelector("ETS2") == GameKind::Ets2);
    CHECK(gameKindFromSelector("eurotrucks2.exe") == GameKind::Ets2);
}

TEST_CASE("ATS and ETS2 keep separate map roots and cache files") {
    CHECK(std::string(gameDefinition(GameKind::Ats).mapName) == "usa");
    CHECK(std::string(gameDefinition(GameKind::Ats).cacheFile) == "map_usa.cache");
    CHECK(std::string(gameDefinition(GameKind::Ets2).mapName) == "europe");
    CHECK(std::string(gameDefinition(GameKind::Ets2).cacheFile) == "map_europe.cache");
}
