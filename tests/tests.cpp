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


#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>


#include "tests_helper.hpp"
#include <gcd/gcode_interpreter.hpp>
#include <lodepng/lodepng.h>
#include <hardware/stepping.hpp>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;

std::vector<std::vector<int>> simulate_moves_on_image(const program_t& prg, const block_t& initial_state)
{
    auto state = merge_blocks(initial_state, {{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}});
    std::pair<double, double> range_x = {state['X'], state['X']};
    std::pair<double, double> range_y = {state['Y'], state['Y']};
    for (auto s : prg) {
        range_x.first = std::min(range_x.first, state['X']);
        range_x.second = std::max(range_x.second, state['X']);
        range_y.first = std::min(range_y.first, state['Y']);
        range_y.second = std::max(range_y.second, state['Y']);
        state = merge_blocks(state, s);
        range_x.first = std::min(range_x.first, state['X']);
        range_x.second = std::max(range_x.second, state['X']);
        range_y.first = std::min(range_y.first, state['Y']);
        range_y.second = std::max(range_y.second, state['Y']);
    }
    range_x.first *= 10;
    range_x.second *= 10;
    range_x.second += 11;
    range_y.first *= 10;
    range_y.second *= 10;
    range_y.second += 11;
    std::vector<std::vector<int>> ret(range_y.second - range_y.first,
        std::vector<int>(range_x.second - range_x.first, 0));
    state = merge_blocks(initial_state, {{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}});

    for (auto stt : prg) {
        auto new_state = merge_blocks(state, stt);

        distance_t p0 = block_to_distance_t(state);
        distance_t p1 = block_to_distance_t(new_state);


        distance_t vp = p1 - p0;
        double s = std::sqrt(vp.length2()); ///< summary distance to go
        distance_t vp_v = vp / s;

        //double T = s / v0;
        for (int i = 0; i < s * 10; ++i) {
            auto pos = p0 + vp_v * i;
            ret.at(pos[1]).at(pos[0]) = pos[2];
        }
        state = new_state;
    }
    return ret;
}

std::vector<std::vector<int>> simulate_moves_on_image(const raspigcd::hardware::multistep_commands_t& prg_steps,
    raspigcd::hardware::motor_layout& motor_layout)
{
    std::pair<double, double> range_x = {0,0};
    std::pair<double, double> range_y = {0,0};
    auto prg = raspigcd::hardware::hardware_commands_to_steps(prg_steps);
    for (auto s : prg) {
        auto p = motor_layout.steps_to_cartesian(s);
        range_x.first = std::min(range_x.first, p[0]);
        range_x.second = std::max(range_x.second, p[0]);
        range_y.first = std::min(range_y.first, p[1]);
        range_y.second = std::max(range_y.second, p[1]);
    }
    range_x.first *= 10;
    range_x.second *= 10;
    range_x.second += 11;
    range_y.first *= 10;
    range_y.second *= 10;
    range_y.second += 11;
    std::vector<std::vector<int>> ret(range_y.second - range_y.first,
        std::vector<int>(range_x.second - range_x.first, 0));
    distance_t state = {};
    for (auto stt : prg) {
        auto new_state = motor_layout.steps_to_cartesian(stt);

        distance_t p0 = state;
        distance_t p1 = new_state;


        distance_t vp = p1 - p0;
        double s = std::sqrt(vp.length2()); ///< summary distance to go
        distance_t vp_v = vp / s;

        for (int i = 0; i < s * 10; ++i) {
            auto pos = p0 + vp_v * i;
            ret.at(pos[1]).at(pos[0]) = pos[2];
        }
        state = new_state;
    }
    return ret;
}


int image_difference(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b)
{
    int s = 0;
    for (int y = a.size(); y--;) {
        for (int x = a[y].size(); x--;) {
            if ((a.at(y).at(x) - b.at(y).at(x)) != 0) {
                std::cout << "diff at: "
                          << x << " " << y
                          << ":: " << a.at(y).at(x) << "  ?  " << b.at(y).at(x) << " :: "
                          << std::endl;
            }
            s += a.at(y).at(x) - b.at(y).at(x);
        }
    }
    return s;
}

std::vector<std::vector<int>> load_image(std::string filename)
{
    std::vector<unsigned char> buffer, image;
    lodepng::load_file(buffer, filename);
    unsigned w, h;
    unsigned error = lodepng::decode(image, w, h, buffer);
    if (error) std::cerr << "error loading image " << filename << std::endl;
    std::vector<std::vector<int>> ret;
    ret.reserve(h);
    for (unsigned int y = 0; y < h; y++) {
        std::vector<int> row;
        row.reserve(w);
        for (unsigned int x = 0; x < w; x++) {
            auto ptr = y * w * 4 + x * 4;
            row.push_back(
                image.at(ptr + 0) +
                (image.at(ptr + 1) << 8) +
                (image.at(ptr + 2) << 16) +
                (image.at(ptr + 3) << 24));
        }
        ret.push_back(row);
    }
    return ret;
}
void save_image(const std::string filename, const std::vector<std::vector<int>>& img_dta)
{
    std::vector<unsigned char> image;
    if (img_dta.size() <= 0) throw std::invalid_argument("image must not be empty");
    if (img_dta[0].size() <= 0) throw std::invalid_argument("image must not be empty");
    int width = img_dta[0].size();
    int height = img_dta.size();

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            image.push_back((img_dta.at(y).at(x) >> 0) & 0x0ff);
            image.push_back((img_dta.at(y).at(x) >> 8) & 0x0ff);
            image.push_back((img_dta.at(y).at(x) >> 16) & 0x0ff);
            image.push_back((img_dta.at(y).at(x) >> 24) & 0x0ff);
        }
    }

    unsigned error = lodepng::encode(filename, image, width, height);
    if (error) throw std::invalid_argument(lodepng_error_text(error));
}
