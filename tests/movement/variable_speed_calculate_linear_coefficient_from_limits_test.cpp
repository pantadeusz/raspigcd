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


#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <hardware/driver/inmem.hpp>

#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
// using namespace raspigcd::movement;
using namespace raspigcd::configuration;

TEST_CASE( "movement variable speed calculate_linear_coefficient_from_limits", "[movement][calculate_linear_coefficient_from_limits][todo]" ) {
	const std::vector<double> limits_for_axes = {1,2,3};
	
	SECTION("1 on each axis") {
		double expected = 1;
		for (const distance_t norm_vect : std::vector<distance_t>{{1,0,0},{0,1,0},	{0,0,1}}) {
			double result = calculate_linear_coefficient_from_limits(limits_for_axes, norm_vect);
			REQUIRE(result == Approx(expected));
			expected++;
		}
	}
}
