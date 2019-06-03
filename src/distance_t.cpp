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


#include <array>
#include <cassert>
#include <cmath>
#include <distance_t.hpp>
#include <iostream>
#include <list>
#include <tuple>
#include <vector>

#include <future>
#include <numeric>

namespace raspigcd {

template <class T, std::size_t N>
T generic_position_t<T, N>::sumv() const
{
    return std::accumulate(this->begin(), this->end(), 0.0);
}

template <std::size_t N>
void follow_path_with_velocity(
    const std::vector<generic_position_t<double, N>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, N>& position)> on_point,
    const double dt,
    const double min_velocity
) {
    if (path_points_with_velocity.size() > 0) {
        auto pos = path_points_with_velocity.front();
        double curr_dist = 0.0;
        auto current_velocity = pos.back(); 
        generic_position_t<double, N> ndistv;
        for (unsigned i = 0; i < path_points_with_velocity.size();) {
            if (current_velocity < min_velocity) {
                std::cerr << "beizer_spline: velocity too small [" << i << "] " << pos << std::endl;
                current_velocity = min_velocity;
            }
            double target_dist = current_velocity * dt; // s = v * t
            ndistv = path_points_with_velocity[i] - pos; // distance to go between points
            ndistv.back() = 0;
            double ndist = ndistv.length();
            if ((curr_dist + ndist) >= target_dist) {
                auto mv = (ndistv / ndist) * (target_dist - curr_dist);
                pos = pos + mv;
                
                auto segment_vect = path_points_with_velocity[i] - path_points_with_velocity[i-1];
                auto dist_to_first = pos - path_points_with_velocity[i-1];
                auto dist_to_second = path_points_with_velocity[i] - pos;
                dist_to_first.back() = 0;
                dist_to_second.back() = 0;
                segment_vect.back() = 0;
                pos.back() = (dist_to_first.length()*path_points_with_velocity[i].back() + 
                             dist_to_second.length()*path_points_with_velocity[i-1].back())/
                             segment_vect.length();


                //pos.back() = path_points_with_velocity[i].back(); // velocity
                
                current_velocity = pos.back();
                on_point(pos);
                curr_dist = 0.0;
            } else {
                curr_dist += ndist;
                pos = path_points_with_velocity[i];
                current_velocity = pos.back();
                i++;
            }
        }
        //        on_point(pos);
    }
}


template <std::size_t N>
void beizer_spline(const std::vector<generic_position_t<double, N>>& path,
    std::function<void(const generic_position_t<double, N>& position)> on_point,
    const double dt,
    const double arc_l,
    const bool velocity_included)
{
    std::vector<std::vector<generic_position_t<double, N>>> triss;
    if (path.size() <= 3) {
        triss.push_back(path);
    } else {
        auto additional_p = [&arc_l,&velocity_included](auto& a0, auto& b, auto& c0) {
            auto ba0 = (b - a0);
            auto const ba0l = ba0.length();
            auto bc0 = (b - c0);
            auto const bc0l = bc0.length();

            auto a = b - ((ba0l != 0.0) ? (ba0 / ba0l) : (ba0 * 0.0));
            auto c = b - ((bc0l != 0.0) ? (bc0 / bc0l) : (bc0 * 0.0));

            if (b == a0) {
                auto vvv = c0 - b;
                auto vvvl = vvv.length();
                auto e = (vvvl > 0.0) ? (b + vvv * std::min(std::abs(arc_l), std::abs(bc0l)) / vvvl) : b;
                // if (velocity_included) {
                //     e.back() = b.back();
                // }
                return std::make_pair(a0, e);
            } else if (b == c0) {
                auto vvv = b - a0;
                auto vvvl = vvv.length();
                auto e = (vvvl > 0.0) ? (b + vvv * std::min(std::abs(arc_l), std::abs(ba0l)) / vvvl) : b;
                // if (velocity_included) {
                //     e.back() = b.back();
                // }
                return std::make_pair(e, b);
            } else {
                auto projv = b - b.projection(a, c);
                auto d = a + projv;
                auto e = c + projv;
                auto vvv = d - e;
                auto vvvl = vvv.length();
                d = b + vvv * std::min(std::abs(arc_l), std::abs(ba0l)) / vvvl;
                e = b - vvv * std::min(std::abs(arc_l), std::abs(bc0l)) / vvvl;
                // if (velocity_included) {
                //     e.back() = b.back();
                //     d.back() = b.back();
                // }
                return std::make_pair(d, e);
            }
        };

        // {
        //   auto a = path[0];
        //   auto b = path[1];
        //   auto c = path[2];
        //   auto [d, e] = additional_p(a, b, c);
        //   if (velocity_included) d.back()=b.back();
        //   triss.push_back({a, d, b, e});
        // }
        for (unsigned i = 1; i < path.size(); i++) {
            std::vector<generic_position_t<double, N>> t;
            {
                i--;
                auto a = path[(i > 0) ? (i - 1) : i];
                auto b = path[i];
                auto c = path[((i + 1) < path.size()) ? (i + 1) : i];
                if (velocity_included) {
                  a.back() = b.back() = c.back() = 0.0;
                }
                auto [d, e] = additional_p(a, b, c);
                e.back() = path[i].back();
                t.push_back(path[i]);
                t.push_back(e);
                //std::cerr << d << " -> " << e << std::endl;
            }
            {
                i++;
                auto a = path[(i > 0) ? (i - 1) : i];
                auto b = path[i];
                auto c = path[((i + 1) < path.size()) ? (i + 1) : i];
                if (velocity_included) {
                  a.back() = b.back() = c.back() = 0.0;
                }
                auto [d, e] = additional_p(a, b, c);
                d.back() = path[i].back();
                t.push_back(d);
                t.push_back(path[i]);
                //std::cerr << d << " -> " << e << std::endl;
            }
            //triss.push_back({a, triss.back().back(), d, b, e});
            triss.push_back(t);
        }
        // {
        //   int i = path.size() - 2;
        //   auto a = path[i];
        //   auto b = path[i + 1];
        //   triss.push_back({a, triss.back().back(), b});
        // }

        std::cerr << std::endl;
    }

    double t = 0;
    std::list<generic_position_t<double, N>> bezier_points;
    for (unsigned i = 0; i < triss.size(); i++) {
        std::vector<generic_position_t<double, N>>& p = triss[i];

        if (p.size() > 4)
            p.resize(4);

        generic_position_t<double, N> pt;
        double l = 0.000001;
        for (unsigned i = 1; i < p.size(); i++) {
            l += (p[i - 1] - p[i]).length();
        }

        std::cerr << "drawing spline of " << p.size() << ": ";
        for (auto e : p) {
            std::cerr << "[";
            for (auto x : e) {
                std::cerr << x << ",";
            }
            std::cerr << "] ";
        }
        std::cerr << std::endl;
        double dt_p = dt / l;
        if (dt_p < 0.0001) dt_p = 0.0001;
        for (; t <= 1.0;) {
            bezier_points.push_back(pt = bezier(p, t));
            if (bezier_points.size() > 1024 * 1024 * 128) {
                std::cerr << "beizer_spline: bezier_points too big: " << bezier_points.size() << " dt_p=" << dt_p << "; t=" << t << std::endl;
                throw std::invalid_argument("bezier_spline: too big bezier_points");
            }
            //double range = std::max(1.0 - t, 0.1) / 2.0;
            //double tp = t + range;
            t += dt_p; //*pt.back()
        }
        t = t - 1.0;
        //std::cerr << "t1 " << t << std::endl;
    }

    if (bezier_points.size() > 0) {
        double curr_dist = 0.0;
        std::vector<generic_position_t<double, N>> bcurve(bezier_points.begin(), bezier_points.end());
        bezier_points.clear();
        auto pos = bcurve.front();
        generic_position_t<double, N> ndistv;
        for (unsigned i = 0; i < bcurve.size();) {
            if (bcurve[i].back() < 0.025) {
                std::cerr << "beizer_spline: velocity too small " << bcurve[i] << std::endl;
                bcurve[i].back() = 0.01;
            }
            double target_dist = bcurve[i].back() * dt;
            ndistv = bcurve[i] - pos;
            ndistv.back() = 0;
            double ndist = ndistv.length();
            if ((curr_dist + ndist) >= target_dist) {
                auto mv = (ndistv / ndist) * (target_dist - curr_dist);
                pos = pos + mv;
                on_point(pos);
                curr_dist = 0.0;
            } else {
                curr_dist += ndist;
                pos = bcurve[i];
                i++;
            }
        }
        //        on_point(pos);
    }
}


template <class T>
void optimize_generic_path_dp_inner(double epsilon, int start, int end, const std::vector<T>& path, std::vector<char>& to_delete)
{
    if (end == -1) end = path.size() -1;
    assert(to_delete.size() == path.size());
    double dmax = 0;
    int index = 0;
    for (int i = start + 1; i < end; i++) {
        if (!to_delete[i]) {
            auto d = point_segment_distance_3d(path[i], path[start], path[end]);
            if (d > dmax) {
                index = i;
                dmax = d;
            }
        }
    }
    if (dmax > epsilon) {
        optimize_generic_path_dp_inner(epsilon, start, index, path, to_delete);
        optimize_generic_path_dp_inner(epsilon, index, end, path, to_delete);
    } else {
        if (start == end)
            return;
        else {
            for (int i = start + 1; i < end; i++) {
                to_delete[i] = true;
            }
        }
    }
};


template <class T>
std::vector<char> optimize_generic_path_dp(double epsilon, const std::vector<T>& path) {
  std::vector<char> to_delete(path.size(), false);
  //DouglasPeucker algorithm
  optimize_generic_path_dp_inner(epsilon, 0, path.size() - 1, path, to_delete);
  return to_delete;
}

template <class T>
std::vector<T> optimize_path_dp(std::vector<T>& path, double epsilon)
{
    std::vector<char> to_delete = optimize_generic_path_dp( epsilon, path);
    std::vector<T> ret;
    ret.reserve(path.size());
    for (unsigned i = 0; i < path.size(); i++) {
        if (!to_delete[i]) {
            ret.push_back(path[i]);
        } else {
            //std::cerr << "Kasuje wezel " << i << " to znaczy " << path[i] << std::endl;
        }
    }
    ret.shrink_to_fit();
    return ret;
}

/// instantiate templates

template int generic_position_t<int, 5>::sumv() const;
template int generic_position_t<int, 4>::sumv() const;
template int generic_position_t<int, 3>::sumv() const;
template int generic_position_t<int, 2>::sumv() const;

template double generic_position_t<double, 5>::sumv() const;
template double generic_position_t<double, 4>::sumv() const;
template double generic_position_t<double, 3>::sumv() const;
template double generic_position_t<double, 2>::sumv() const;


template void beizer_spline<2>(const std::vector<generic_position_t<double, 2>>& path,
    std::function<void(const generic_position_t<double, 2>& position)> on_point,
    const double dt,
    const double arc_l,
    const bool velocity_included);


template void beizer_spline<3>(const std::vector<generic_position_t<double, 3>>& path,
    std::function<void(const generic_position_t<double, 3>& position)> on_point,
    const double dt,
    const double arc_l,
    const bool velocity_included);

template void beizer_spline<4>(const std::vector<generic_position_t<double, 4>>& path,
    std::function<void(const generic_position_t<double, 4>& position)> on_point,
    const double dt,
    const double arc_l,
    const bool velocity_included);

template void beizer_spline<5>(const std::vector<generic_position_t<double, 5>>& path,
    std::function<void(const generic_position_t<double, 5>& position)> on_point,
    const double dt,
    const double arc_l,
    const bool velocity_included);


template std::vector<generic_position_t<double,2>> optimize_path_dp<generic_position_t<double,2>>(std::vector<generic_position_t<double,2>>& path, double epsilon);
template std::vector<generic_position_t<double,3>> optimize_path_dp<generic_position_t<double,3>>(std::vector<generic_position_t<double,3>>& path, double epsilon);
template std::vector<generic_position_t<double,4>> optimize_path_dp<generic_position_t<double,4>>(std::vector<generic_position_t<double,4>>& path, double epsilon);
template std::vector<generic_position_t<double,5>> optimize_path_dp<generic_position_t<double,5>>(std::vector<generic_position_t<double,5>>& path, double epsilon);
template std::vector<generic_position_t<double,6>> optimize_path_dp<generic_position_t<double,6>>(std::vector<generic_position_t<double,6>>& path, double epsilon);
template std::vector<char > 
optimize_generic_path_dp<raspigcd::generic_position_t<double, 5ul> >
(double, std::vector<generic_position_t<double, 5ul>, std::allocator<generic_position_t<double, 5ul> > > const&);



template void follow_path_with_velocity<2>(const std::vector<generic_position_t<double, 2>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, 2>& position)> on_point, double dt,
    const double min_velocity
);
template void follow_path_with_velocity<3>(const std::vector<generic_position_t<double, 3>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, 3>& position)> on_point, double dt,
    const double min_velocity
);
template void follow_path_with_velocity<4>(const std::vector<generic_position_t<double, 4>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, 4>& position)> on_point, double dt,
    const double min_velocity
);
template void follow_path_with_velocity<5>(const std::vector<generic_position_t<double, 5>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, 5>& position)> on_point, double dt,
    const double min_velocity
);

} // namespace raspigcd
