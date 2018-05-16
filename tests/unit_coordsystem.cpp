#include "coordsystem.hpp"
#include "coordsystem_corexy.hpp"
#include "coordsystem_euclidean.hpp"

#include <catch.hpp>
#include <fakeit.hpp>

#include <iostream>
#include <vector>

using namespace tp::motor;

using namespace fakeit;

using namespace tp::coord;
using namespace tp::motor;

TEST_CASE("coordsystem simple test", "[coordsystem][CoordTranslateSimple]")
{
    CoordTranslateSimple simpleTranslate({ 100, 200, 300, 300 }, Position(1, 1, 1, 1));
    Position pos(10.5, -10.5, 30.1, 30.1);
    Steps steps(1001, 2001, 123, 123);
    SECTION("simple movement always positive")
    {
        steps = simpleTranslate.translate(pos);
        REQUIRE(steps[0] == pos[0] * 100.0);
        REQUIRE(steps[1] == pos[1] * 200.0);
        REQUIRE(steps[2] == pos[2] * 300.0);
        REQUIRE(steps[3] == pos[3] * 300.0);
    }
    SECTION("simple movement always positive")
    {
        pos = simpleTranslate.translate(steps);
        REQUIRE(pos[0] == steps[0] / 100.0);
        REQUIRE(pos[1] == steps[1] / 200.0);
        REQUIRE(pos[2] == steps[2] / 300.0);
        REQUIRE(pos[3] == steps[3] / 300.0);
    }
    SECTION("f(g(x))=x?")
    {
        auto pos2 = simpleTranslate.translate(simpleTranslate.translate(pos));
        REQUIRE(pos[0] == pos2[0]);
        REQUIRE(pos[1] == pos2[1]);
        REQUIRE(pos[2] == pos2[2]);
        REQUIRE(pos[3] == pos2[3]);
    }
}

TEST_CASE("coordsystem core xy test", "[coordsystem][CoordTranslateCoreXY]")
{
    CoordTranslateCoreXY simpleTranslate({ 100, 200, 300, 100 }, Position(1, 1, 1, 1));
    Position pos(10.5, -10.5, 30.1, 10);
    Steps steps(1001, 2001, 123, 1000);
    SECTION("f(g(x))=x?")
    {
        auto pos2 = simpleTranslate.translate(simpleTranslate.translate(pos));
        REQUIRE(pos[0] == pos2[0]);
        REQUIRE(pos[1] == pos2[1]);
        REQUIRE(pos[2] == pos2[2]);
        REQUIRE(pos[3] == pos2[3]);
    }
}
