/*
    Raspberry Pi G-CODE interpreter

    Copyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <distance_t.hpp>
#include <thread>
#include <vector>

using namespace raspigcd;

TEST_CASE("distance_t - check vector operations", "[common][distance_t]")
{
    SECTION("when two points are the same relative to the distance_t, then the angle should be 0")
    {
        distance_t middle = {100, 200, 150, 50};
        distance_t a = {130, 200, 150, 50};
        distance_t b = {130, 200, 150, 50};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == 0);
    }
    SECTION("when two points are the same line relative to the distance_t, then the angle should be M_PI")
    {
        distance_t middle = {100, 200, 150, 50};
        distance_t a = {100 + 10, 200 + 20, 150 + 5, 50 + 1};
        distance_t b = {100 - 10, 200 - 20, 150 - 5, 50 - 1};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI));
    }
    SECTION("90 deg test should result in M_PI/2")
    {
        distance_t middle = {0, 0, 0, 0};
        distance_t a = {0, 1, 0, 0};
        distance_t b = {1, 0, 0, 0};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI / 2));
    }
    SECTION("90 deg test should result in M_PI/2")
    {
        distance_t middle = {0, 0, 0, 0};
        distance_t a = {0, -1, 0, 0};
        distance_t b = {1, 0, 0, 0};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI / 2));
    }
    SECTION("90 deg test should result in M_PI/2 also in reverse order")
    {
        distance_t middle = {0, 0, 0, 0};
        distance_t b = {0, -1, 0, 0};
        distance_t a = {1, 0, 0, 0};

        auto ret = middle.angle(a, b);
        REQUIRE(ret == Approx(M_PI / 2));
    }
    //SECTION("180 deg test should result in M_PI")
    //{
    //    distance_t a = {143.849,-17.018,0,0};
    //    distance_t middle = {144.611,-19.4733,0,0};
    //    distance_t b = {144.526,-23.4526,0,0};
    //
    //    auto ret = middle.angle(a,b);
    //    REQUIRE(ret == Approx(M_PI));
    //}
}
