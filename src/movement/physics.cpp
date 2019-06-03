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
#include <distance_t.hpp>
#include <movement/physics.hpp>
//#include <hardware/stepping_commands.hpp>
#include <list>
#include <steps_t.hpp>
#include <cmath>

namespace raspigcd {
namespace movement {

/**
 * @brief the physics library that can be used to calculate accelerations, distances and other things.
 */
namespace physics {

distance_t get_next_position(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return s0;
    distance_t a_= (v0/l)*a;
    distance_t s1 = s0 + v0 * t + a_*t*t/2.0;
    return s1;
}

distance_t get_next_velocity( const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return v0;
    distance_t a_= (v0/l)*a;
    return v0 + a_*t;
}

path_node_t get_next_node(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return {s0,std::sqrt(v0.length2())};

    const auto& [s1,v1] = get_next_s_v(s0, v0, a,t);

    auto v0s = std::sqrt((v0).length2());
    auto v1s = std::sqrt((v1).length2());
    if (v1s == 0.0) return {s1,0.0};
    auto v0norm = v0/v0s;
    auto v1norm = v1/v1s;
    auto vdiff = (int)std::sqrt((v1norm-v0norm).length2());
    double d = (1.0 - vdiff);
    return {s1,(d*v1s)};
}

std::pair<distance_t,distance_t> get_next_s_v(const distance_t &s0, const distance_t &v0, const double &a, const double &t) {
    auto l = std::sqrt(v0.length2());
    if (l == 0.0) return {s0,v0};
    distance_t a_= (v0/l)*a;
    distance_t s1 = s0 + v0 * t + a_*t*t/2.0;
    auto v1 = v0 + a_*t;
    
    return {s1,v1};
}



double acceleration_between(const path_node_t &a, const path_node_t &b) {
    auto s = (b.p-a.p).length();
    auto dv = (b.v-a.v);
    if (dv == 0) return 0;
    if (s == 0) throw std::invalid_argument("cannot do infinite accelerations");
    
    double a_min = -10000000.0, a_max= 10000000.0;
    
    #ifndef NDEBUG
    double tt = 0;
    #endif
    for (int n = 0; n < 82; n++) {
        double acc = (a_max + a_min)/2.0;
        double t = 0.0;
        if (acc != 0.0) {
            t = std::abs((b.v-a.v)/acc);
            #ifndef NDEBUG
            tt = t;
            #endif
            double s1 = a.v * t + acc*t*t/2.0;
            //std::cout << "********s1 = " << s1 << " ?? " << s  << "  acc: " << acc << std::endl;
            double mx = (dv >= 0)?1:-1;
            if (mx*s1 > mx*s) {
                a_min = acc;
            } else if (mx*s1 < mx*s) {
                a_max = acc;
            } else if (s1 == s) {
                //std::cout << "********n = " << n << std::endl;
                return acc;
            }
        } else if ((b.v-a.v) > 0) {
            a_min = acc;
        } else if ((b.v-a.v) < 0) {
            a_max = acc;
        } else {
            //std::cout << "********acc = " << 0 << std::endl;
            return 0.0;
        }
    }
    #ifndef NDEBUG
    // std::cout << "WARNING: not precise result: double acceleration_between(const path_node_t &a, const path_node_t &b) : s: " << s <<" dv " << dv << "    tt " << tt << std::endl;
    #endif
    return (a_max + a_min)/2.0;
}

path_node_t calculate_transition_point(const path_node_t &a, const path_node_t &b, const double acceleration) {
    if (acceleration == 0) return a;
    if (a.p == b.p) {
        if (a.v != b.v) throw std::invalid_argument("cannot calculate transition point for two same points");
        else return a;
    }
    auto road_vect = b.p-a.p;
    auto s_target = (road_vect).length();
    double vf2 = a.v * a.v + 2*acceleration*s_target;
    path_node_t ret = b;
    if (vf2 > 0) ret.v = std::sqrt(vf2);
    if (((ret.v > b.v) && (acceleration > 0)) ||
       ((vf2 < 0) && (acceleration < 0))) {
        //std::cout << "A: v final " << (ret.v) << " pos: " <<ret.p << std::endl;
        // v=s/t
        // a = v2-v1/t => t = v2-v1/a
        double t = (b.v-a.v)/acceleration;
        //std::cout << t << std::endl;
        double s = a.v * t + acceleration*t*t/2.0;
        ret.v = b.v;
        ret.p = a.p + (b.p-a.p)/(b.p-a.p).length() *s;
        
    }
//    std::cout << "B: v final " << (ret.v) << " pos: " <<ret.p << std::endl;
    return ret;
}


bool operator==(const path_node_t &lhs,const path_node_t &rhs) {
    if ((lhs.p == rhs.p) && (lhs.v == rhs.v)) return true;
    return false;
}



} // namespace physics

} // namespace movement
} // namespace raspigcd

