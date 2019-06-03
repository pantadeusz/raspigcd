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


#ifndef __RASPIGCD_MOVEMENT_PHYSICS_HPP__
#define __RASPIGCD_MOVEMENT_PHYSICS_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <steps_t.hpp>

namespace raspigcd {

/**
 * the namespace that contains everything aboult movememt - the physics, the steps generation and analyze.
 * */
namespace movement {

/**
 * @brief the physics library that can be used to calculate accelerations, distances and other things.
 *
 * basic units are: mm, s
 */

namespace physics {


/**
 * The simplified position and velocity. This is used in calculations. Can be taken from machine state.
 * */
struct path_node_t {
    distance_t p; ///< position
    double v;     ///< velocity
};

/*
v1 = v0 + a*t
s1 = s0 + v0*t + a*t*t/2
*/

/**
 * @brief calculate velocity for given distance and acceleration.
 */
distance_t get_next_position(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

/**
 * @brief returns next velocity
 */
distance_t get_next_velocity(const distance_t &v0, const double &a, const double &t);

/**
 * @brief returns next velocity
 */
path_node_t get_next_node(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

/**
 * This method returns position and velocity in given direction after execution of movement from point s0 with initial velocity
 * v0 and linear acceleration a.
 * 
 * @param s0 initial position
 * @param v0 initial velocity vector
 * @param a acceleration (can be negative)
 * @param t time of the movement
 * 
 * @return position and velocity vector after the given time 
 * */
std::pair<distance_t,distance_t> get_next_s_v(const distance_t &s0, const distance_t &v0, const double &a, const double &t);

/**
 * @brief calculates acceleration for given points and speeds
 */
double acceleration_between(const path_node_t &a, const path_node_t &b);

/**
 * gets the point at wich the movement will reach velocity of the second node.
 * */
path_node_t calculate_transition_point(const path_node_t &a, const path_node_t &b, const double acceleration);


bool operator==(const path_node_t &lhs,const path_node_t &rhs);

} // namespace physics

} // namespace movement
} // namespace raspigcd

#endif
