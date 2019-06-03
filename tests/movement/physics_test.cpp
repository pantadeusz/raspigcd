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

#include <configuration.hpp>
#include <configuration_json.hpp>
#include <movement/physics.hpp>


#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
//using namespace raspigcd::hardware;
using namespace raspigcd::movement::physics;

// distance_t size_maker_distance;
const unsigned COORDINATES_COUNT = 3;

auto cmp_approx = [](auto lhs, auto rhs) {
    for (unsigned i = 0; i < lhs.size(); i++) {
        if (lhs.at(i) != Approx(rhs.at(i))) {
            INFO(lhs);
            INFO(rhs);
            FAIL("Difference at index " + std::to_string(i)); // + " values=(" + lhs + " != " + rhs + ")");
        }
    }
    return true;
};


TEST_CASE("Movement physics simple formulas get_next_position", "[movement][physics][get_next_position]")
{
    SECTION("trivial next position get_next_position for no acceleration and 0 speed")
    {
        const distance_t s0{10, 11, 12};
        const distance_t v0{0, 0, 0};
        const double a = 0;
        const double t = 2;
        distance_t result = get_next_position(s0, v0, a, t);
        REQUIRE(cmp_approx(result, s0));
    }
    SECTION("get_next_position for no acceleration and speed of 2mm/s in one direction")
    {
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            const distance_t s0{10, 11, 12};
            distance_t v0{0, 0, 0};
            v0[i] = 1.0;
            const double a = 0;
            const double t = 2;
            distance_t result = get_next_position(s0, v0, a, t);
            distance_t expected = s0;
            expected[i] += v0[i] * t;
            REQUIRE(cmp_approx(result, expected));
        }
    }
    SECTION("get_next_position for speed of 2mm/s and acceleration in one direction")
    {
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            const distance_t s0{10, 11, 12};
            distance_t v0{0, 0, 0};
            v0[i] = 1.0;
            const double a = 2;
            const double t = 2;
            INFO(v0);
            INFO(a);
            distance_t result = get_next_position(s0, v0, a, t);
            distance_t expected = s0;
            expected[i] += v0[i] * t + a * t * t / 2.0;
            REQUIRE(cmp_approx(result, expected));
        }
    }

    SECTION("get_next_position for speed of 2mm/s and negative acceleration in all directions")
    {
        const distance_t s0{10, 11, 12};
        distance_t v0{3.0, 4.0, 0.0};
        const double a = -1;
        const double t = 5;
        INFO(v0);
        INFO(a);
        auto end_pos = get_next_position(s0, v0, a, t);
        INFO(end_pos);
        double result = std::sqrt((end_pos - s0).length2());
        double expected = std::sqrt(v0.length2()) * t + a * t * t / 2.0;
        REQUIRE(result == Approx(expected));
    }
}
// distance_t get_next_velocity(const distance_t &s0, const distance_t &v0, const double &a, const double &t)

TEST_CASE("Movement physics simple formulas get_next_velocity", "[movement][physics][get_next_velocity]")
{
    SECTION("trivial next position get_next_velocity for no acceleration and 0 speed")
    {
        const distance_t s0{10, 11, 12};
        const distance_t v0{0, 0, 0};
        const double a = 0;
        const double t = 2;
        distance_t result = get_next_velocity(v0, a, t);
        REQUIRE(cmp_approx(result, v0));
    }

    SECTION("get_next_velocity for no acceleration and speed of 2mm/s in one direction")
    {
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            const distance_t s0{10, 11, 12};
            distance_t v0{0, 0, 0};
            v0[i] = 1.0;
            const double a = 0;
            const double t = 2;
            distance_t result = get_next_velocity(v0, a, t);
            distance_t expected = v0;
            REQUIRE(cmp_approx(result, expected));
        }
    }

    SECTION("get_next_velocity for speed of 2mm/s and acceleration in one direction")
    {
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            const distance_t s0{10, 11, 12};
            distance_t v0{0, 0, 0};
            v0[i] = 10.0;
            const double a = -5;
            const double t = 2;
            INFO(v0);
            INFO(a);
            distance_t result = get_next_velocity(v0, a, t);
            distance_t expected = v0;
            expected[i] += a * t;
            REQUIRE(cmp_approx(result, expected));
        }
    }

    SECTION("get_next_velocity for speed of 2mm/s and negative acceleration in all directions")
    {
        const distance_t s0{10, 11, 12};
        distance_t v0{3.0, 4.0, 0.0};
        const double a = -1;
        const double t = 5;
        INFO(v0);
        INFO(a);
        auto end_v = get_next_velocity(v0, a, t);
        INFO(end_v);
        double result = std::sqrt(end_v.length2());
        double expected = 0.0;
        INFO(result);
        REQUIRE(result == Approx(expected).margin(0.00000000000001));
    }
}

TEST_CASE("Movement physics simple formulas get_next_node", "[movement][physics][get_next_node]")
{
    SECTION("get_next_node for speed of 2mm/s and acceleration in one direction")
    {
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            const distance_t s0{10, 11, 12};
            distance_t v0{0, 0, 0};
            v0[i] = 10.0;
            const double a = -5;
            const double t = 2;
            INFO(v0);
            INFO(a);
            path_node_t node = get_next_node(s0, v0, a, t);
            distance_t expected_v1 = v0;
            expected_v1[i] += a * t;
            REQUIRE(node.v == std::sqrt(expected_v1.length2()));
            distance_t expected_s1 = s0;
            expected_s1[i] += v0[i] * t + a * t * t / 2.0;
            REQUIRE(cmp_approx(node.p, expected_s1));
        }
    }

    SECTION("get_next_node for speed of 2mm/s and negative acceleration in all directions")
    {
        const distance_t s0{10, 11, 12};
        distance_t v0{3.0, 4.0, 0.0};
        const double a = -1;
        const double t = 5;
        INFO(v0);
        INFO(a);
        path_node_t node = get_next_node(s0, v0, a, t);
        INFO(node.p);
        INFO(node.v);
        REQUIRE(node.v == Approx(0.0));
    }
    SECTION("get_next_node for speed of 2mm/s and negative acceleration in all directions")
    {
        const distance_t s0{10, 11, 12};
        distance_t v0{3.0, 4.0, 0.0};
        const double a = -1;
        const double t = 2;
        INFO(v0);
        INFO(a);
        path_node_t node = get_next_node(s0, v0, a, t);
        INFO(node.p);
        INFO(node.v);
        REQUIRE(node.v == Approx(3.0));
    }
    SECTION("get_next_node for speed of 2mm/s and negative acceleration that results in negative speed")
    {
        const distance_t s0{10, 11, 12};
        distance_t v0{3.0, 4.0, 0.0};
        const double a = -1;
        const double t = 10;
        INFO(v0);
        INFO(a);
        path_node_t node = get_next_node(s0, v0, a, t);
        INFO(node.p);
        INFO(node.v);
        REQUIRE(node.v == Approx(-5.0));
    }
}


TEST_CASE("Movement physics simple formulas acceleration_between", "[movement][physics][acceleration_between]")
{
    SECTION("acceleration_between with no acceleration")
    {
        distance_t p0 = {3.0, 4.0, 0.0}, p1 = {6.0, 8.0, 0.0};
        path_node_t node_a = {.p = p0, 1};
        double t = 5;
        double a = 0.0;
        auto v = ((p1 - p0) / (p1 - p0).length()) * node_a.v;
        INFO(v);
        path_node_t node_b = get_next_node(node_a.p, v, a, t);
        INFO(node_a.p);
        INFO(node_a.v);
        INFO(node_b.p);
        INFO(node_b.v);

        double accel_v = acceleration_between(node_a, node_b);
        REQUIRE(accel_v == Approx(0.0));
    }

    SECTION("acceleration_between with acceleration")
    {
        distance_t p0 = {3.0, 4.0, 0.0}, p1 = {8.25, 11, 0};
        path_node_t node_a = {.p = p0, 1};
        double t = 2.5;
        double a = 2.0;
        double s = 8.75;
        INFO(p1);
        p1 = p0 + (p1 - p0) * s / (p1 - p0).length();
        INFO(p1);
        auto v = ((p1 - p0) / (p1 - p0).length()) * node_a.v;
        INFO(v);
        path_node_t node_b = get_next_node(node_a.p, v, a, t);
        INFO(node_a.p);
        INFO(node_a.v);
        INFO(node_b.p);
        INFO(node_b.v);
        double accel_v = acceleration_between(node_a, node_b);
        REQUIRE(accel_v == Approx(a));
    }


    SECTION("acceleration_between with acceleration on one axis")
    {
        distance_t p0 = {0.0, 0.0, 0.0}, p1 = {50, 0, 0};
        path_node_t node_a = {.p = p0, 0};
        double t = 10;
        double a = 1.0;
        double s = 50;
        INFO(p1);
        p1 = p0 + (p1 - p0) * s / (p1 - p0).length();
        INFO(p1);
        auto v = ((p1 - p0) / (p1 - p0).length()) * node_a.v;
        INFO(v);
        path_node_t node_b = {.p = p1, .v = a * t};
        INFO(node_a.p);
        INFO(node_a.v);
        INFO(node_b.p);
        INFO(node_b.v);
        double accel_v = acceleration_between(node_a, node_b);
        REQUIRE(accel_v == Approx(a));
    }

    SECTION("acceleration_between with acceleration on one axis from not zero pos")
    {
        distance_t p0 = {2.0, 0.0, 0.0}, p1 = {52, 0, 0};
        path_node_t node_a = {.p = p0, 0};
        double t = 10;
        double a = 1.0;
        double s = 50;
        INFO(p1);
        p1 = p0 + (p1 - p0) * s / (p1 - p0).length();
        INFO(p1);
        auto v = ((p1 - p0) / (p1 - p0).length()) * node_a.v;
        INFO(v);
        path_node_t node_b = {.p = p1, .v = a * t};
        INFO(node_a.p);
        INFO(node_a.v);
        INFO(node_b.p);
        INFO(node_b.v);
        double accel_v = acceleration_between(node_a, node_b);
        REQUIRE(accel_v == Approx(a));
    }

    SECTION("acceleration_between with acceleration on one axis from not zero vel")
    {
        double u = 1;
        double a = 2.0;
        double t = 3;
        double s = 12;
        distance_t p0 = {0.0, 0.0, 0.0}, p1 = {s, 0, 0};
        path_node_t node_a = {.p = p0, u};
        INFO(p1);
        p1 = p0 + (p1 - p0) * s / (p1 - p0).length();
        INFO(p1);
        auto v = ((p1 - p0) / (p1 - p0).length()) * node_a.v;
        INFO(v);
        path_node_t node_b = {.p = p1, .v = 1 + a * t};
        INFO(node_a.p);
        INFO(node_a.v);
        INFO(node_b.p);
        INFO(node_b.v);
        double accel_v = acceleration_between(node_a, node_b);
        {
            double s = 0;
            double v = 1;
            for (double t_ = 0; t_ < t; t_ += 0.0001) {
                s = s + v * 0.0001 + 0.5 * a * 0.0001 * 0.0001;
                v = v + a * 0.0001; // velocity
            }
            //std::cout << "s = " << s << " v = " << v << std::endl;
            REQUIRE(s == Approx(12.0));
            REQUIRE(v == Approx(7.0));
        }
        {
            double s = 0;
            double v = 1;
            for (double t_ = 0; t_ < t; t_ += 0.0001) {
                s = s + v * 0.0001 + 0.5 * accel_v * 0.0001 * 0.0001;
                v = v + accel_v * 0.0001; // velocity
            }
            //std::cout << "s = " << s << " v = " << v << std::endl;
            REQUIRE(s == Approx(12.0));
            REQUIRE(v == Approx(7.0));
        }
        REQUIRE(accel_v == Approx(a));
    }

    SECTION("acceleration_between with acceleration on one axis 2")
    {
        distance_t p0 = {0.0, 0.0, 0.0}, p1 = {22.5, 0, 0};
        path_node_t node_a = {.p = p0, 0};
        double t = 3;
        double a = 5.0;
        double s = 22.5;
        INFO(p1);
        p1 = p0 + (p1 - p0) * s / (p1 - p0).length();
        INFO(p1);
        auto v = ((p1 - p0) / (p1 - p0).length()) * node_a.v;
        INFO(v);
        path_node_t node_b = {.p = p1, .v = node_a.v + a * t};
        INFO(node_a.p);
        INFO(node_a.v);
        INFO(node_b.p);
        INFO(node_b.v);
        double accel_v = acceleration_between(node_a, node_b);
        REQUIRE(accel_v == Approx(a));
    }
    // TODO: Negative accelerations!
    SECTION("acceleration_between with acceleration on one axis 2 - breaks")
    {
        distance_t p0 = {0.0, 0.0, 0.0}, p1 = {22.5, 0, 0};
        path_node_t node_a = {.p = p0, 20};
        double t = 3;
        double a = -5.0;
        double s = p0[0] + node_a.v*t + a*t*t/2;
        p1[0] = s;
        path_node_t node_b = {.p = p1, .v = node_a.v + a * t};
        double accel_v = acceleration_between(node_a, node_b);
        REQUIRE(accel_v == Approx(a));
    }
}

TEST_CASE("Get final velocity for limits", "[movement][physics][calculate_transition_point]")
{
    
    SECTION("no acceleration should return the first node 1")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 10.0};
        const path_node_t b = {.p = {5,2,3}, .v = 10.0};
        double acceleration = 0.0;
        path_node_t ret = calculate_transition_point(a,b,acceleration);
        REQUIRE(ret == a);
    }
    SECTION("no acceleration should return the first node 2")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 10.0};
        const path_node_t b = {.p = {5,2,3}, .v = 15.0};
        double acceleration = 0.0;
        path_node_t ret = calculate_transition_point(a,b,acceleration);
        REQUIRE(ret == a);
    }
    SECTION("acceleration slow enough should return second node with final velocity")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 10.0};
        const path_node_t b = {.p = {5,2,3}, .v = 50.0};
        double acceleration = 1.0;
        path_node_t ret = calculate_transition_point(a,b,acceleration);
        
        for (size_t i = 0; i < COORDINATES_COUNT; i++)
            REQUIRE(ret.p[i] == Approx(b.p[i]));
        REQUIRE(ret.v < b.v);
        REQUIRE(ret.v > a.v);

        double accel_v = acceleration_between(a, ret);
        REQUIRE(accel_v == Approx(acceleration));
    }
    SECTION("acceleration slow enough should return second node with final velocity and negative acceleration")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 10.0};
        const path_node_t b = {.p = {5,2,3}, .v = 2.0};
        double acceleration = -1.0;
        path_node_t ret = calculate_transition_point(a,b,acceleration);
        
        for (size_t i = 0; i < COORDINATES_COUNT; i++)
            REQUIRE(ret.p[i] == Approx(b.p[i]));
        //REQUIRE(ret.v < b.v);
        //REQUIRE(ret.v > a.v);

        double accel_v = acceleration_between(a, ret);
        REQUIRE(accel_v == Approx(acceleration));
    }
    SECTION("fast acceleration")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 10.0};
        const path_node_t b = {.p = {5,3,4}, .v = 15.0};
        double acceleration = 100.0;
        path_node_t ret = calculate_transition_point(a,b,acceleration);
        
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            REQUIRE(ret.p[i] < b.p[i]);
            REQUIRE(ret.p[i] > a.p[i]);
        }
        REQUIRE(ret.v == Approx(b.v));

        double accel_v = acceleration_between(a, ret);
        REQUIRE(accel_v == Approx(acceleration));
    }
    SECTION("fast negative acceleration")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 15.0};
        const path_node_t b = {.p = {5,3,4}, .v = 10.0};
        double acceleration = -100.0;
        path_node_t ret = calculate_transition_point(a,b,acceleration);
        
        for (size_t i = 0; i < COORDINATES_COUNT; i++) {
            REQUIRE(ret.p[i] < b.p[i]);
            REQUIRE(ret.p[i] > a.p[i]);
        }
        REQUIRE(ret.v == Approx(b.v));

        double accel_v = acceleration_between(a, ret);
        REQUIRE(accel_v == Approx(acceleration));
    }
    SECTION("exception when both points are the same and there is difference in velocity")
    {
        const path_node_t a = {.p = {1,2,3}, .v = 15.0};
        const path_node_t b = {.p = {1,2,3}, .v = 10.0};
        double acceleration = -100.0;
        REQUIRE_THROWS(calculate_transition_point(a,b,acceleration));
    }
   SECTION("exception when both points are the same and there is difference in velocity")
   {
       const path_node_t a = {.p = {1,2,3}, .v = 15.0};
       const path_node_t b = {.p = {1,2,3}, .v = 10.0};
       double acceleration = -100.0;
       REQUIRE_THROWS(calculate_transition_point(a,b,acceleration));
   }

}