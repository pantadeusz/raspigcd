/*

    This is the gcode generator from image that uses genetic algorithm for optimization of path
    Copyright (C) 2019  Tadeusz Puźniakowski

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.


*/



#ifndef __RASPIGCD_generic_position_t_HPP__
#define __RASPIGCD_generic_position_t_HPP__

#include <list>
#include <numeric>

#include <array>
#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <functional>
#include <cmath>

namespace raspigcd {
template<class T, std::size_t N>
class generic_position_t;

template<class T, std::size_t N>
inline generic_position_t<T,N> operator*(
                    const generic_position_t<T,N>& a,
                    const generic_position_t<T,N>& b);

/**
 * You can also call it coordinates if you want.
 * */

template<class T, std::size_t N>
class generic_position_t : public std::array<T, N>
{
public:
    generic_position_t() : std::array<T, N>(){
        for (auto &e:*this) e=0;
    };
    generic_position_t(std::initializer_list<T> v) : std::array<T, N>()
    {
        std::size_t i = 0;
        for (auto &e:v) {
            if (i >= N) throw std::invalid_argument("you can't put greater number of elements than can be fit in the generic_position_t");
            else this->operator[](i) = e;
            i++;
        }
    };
    generic_position_t(const std::vector<T> &v) : std::array<T, N>()
    {
        for (std::size_t i = 0; i < std:: min(v.size(),this->size()); i++) {
            this->operator[](i) = v[i];
        }
    };

    generic_position_t(const std::array<T, N-1> &v) : std::array<T, N>()
    {
        for (std::size_t i = 0; i < std:: min(v.size(),this->size()); i++) {
            this->operator[](i) = v[i];
        }
    };
    generic_position_t(const std::array<T, N+1> &v) : std::array<T, N>()
    {
        for (std::size_t i = 0; i < std:: min(v.size(),this->size()); i++) {
            this->operator[](i) = v[i];
        }
    };

    inline double length2() const
    {
        T ret = 0.0;
        for (auto &e:*this) ret += e*e;
        return ret;
    }
    inline double length() const
    {
        return std::sqrt(length2());
    }
    /**
     * @brief returns angle between 0 - PI
     * 
     */
    double angle(const generic_position_t & a, const generic_position_t & b) const;

    T sumv() const;

    inline double dot_product(const generic_position_t &b) const {
        const generic_position_t a = *this;
        return (a * b).sumv();
    }

    inline generic_position_t projection(const generic_position_t &v, const generic_position_t &w) const {
      auto &a = v;
      auto &b = w;
      auto &p = *this;
      auto ap = p - a;
      auto ab = b - a;
      return a + ab * ((ap).dot_product(ab) / (ab).dot_product(ab));
    }
};

template<class T,std::size_t N>
inline generic_position_t<T,N> operator+(
    const generic_position_t<T,N>& a,
    const generic_position_t<T,N>& b)
{
    generic_position_t<T,N> ret = a;
    for (std::size_t i = 0; i < N; i++) ret[i] += b[i];
    return ret;
}

template<class T,std::size_t N>
inline generic_position_t<T,N> operator-(
    const generic_position_t<T,N>& a,
    const generic_position_t<T,N>& b)
{
    generic_position_t<T,N> ret = a;
    for (std::size_t i = 0; i < N; i++) ret[i] -= b[i];
    return ret;
}

template<class T,std::size_t N>
inline generic_position_t<T,N> operator*(
    const generic_position_t<T,N>& a,
    const generic_position_t<T,N>& b)
{
    generic_position_t<T,N> ret = a;
    for (std::size_t i = 0; i < N; i++) ret[i] *= b[i];
    return ret;
}

template<class T,std::size_t N>
inline generic_position_t<T,N> operator/(
    const generic_position_t<T,N>& a, 
    const generic_position_t<T,N>& b)
{
    generic_position_t<T,N> ret = a;
    for (std::size_t i = 0; i < N; i++) ret[i] /= b[i];
    return ret;

}

template<class T,std::size_t N>
inline generic_position_t<T,N> operator*(
    const generic_position_t<T,N>& a, const double& b)
{
    generic_position_t<T,N> ret = a;
    for (std::size_t i = 0; i < N; i++) ret[i] *= b;
    return ret;
}



template<class T,std::size_t N>
inline generic_position_t<T,N> operator/(const generic_position_t<T,N>& a, const double& b)
{
    generic_position_t<T,N> ret = a;
    for (std::size_t i = 0; i < N; i++) ret[i] /= b;
    return ret;

}

template<class T,std::size_t N>
inline double generic_position_t<T,N>::angle(
    const generic_position_t<T,N>& a,
    const generic_position_t<T,N>& b) const
{
    auto u = a - (*this);auto ul = u.length();
    auto v = b - (*this);auto vl = v.length();
    if (ul < 0.000001) return 3.14159265358979323846 / 2.0;
    if (vl < 0.000001) return 3.14159265358979323846 / 2.0;
    u = u / ul;
    v = v / vl;
    auto dotprod = (u * v).sumv();
    if (std::isnan(dotprod)) throw std::invalid_argument("dotprod is nan");
    if (std::abs(dotprod) < 0.0001) return 3.14159265358979323846 / 2.0;
    auto acosv = dotprod / (u.length() * v.length());
    if (std::isnan(acosv)) throw std::invalid_argument("acosv is nan");
    if (acosv <= -1.0) return 3.14159265358979323846;
    if (acosv >= 1.0) return 0.0 ; //3.14159265358979323846;//0;
    return acos(acosv);
}

template<class T,std::size_t N>
inline bool operator==(const generic_position_t<T,N>& a, const generic_position_t<T,N>& b)
{
    bool ret = true;
    for (std::size_t i = 0; i < a.size(); i++) ret = ret && (a[i] == b[i]);
    return ret;
}

template<class T,std::size_t N>
inline std::ostream& operator<<(std::ostream& os, generic_position_t<T,N> const& value)
{
    os << "[";
    int i = 0;
    for (auto &e :value) {
        os << ((i==0)?(""):(",")) << e;
        i++;
    }
    os << "]";
    return os;
}


/*
    * calulates maximal linear value given the maximal values for each axis, and the normal vector of intended move
    * it works that if norm_vect is 1 on one axis, then the value from limits_for_axes on this
    * otherwise it blends, so if it is a limit, then applying linear limit will not exceed limits
    * it is used to calculate maximal speed for movement in any direction, as well as maximal acceleration and speed without acceleration
    */
//double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const generic_position_t& norm_vect);

//double calculate_linear_coefficient_from_limits(const std::vector<double>& limits_for_axes, const generic_position_t& norm_vect)
//std::vector<double>
auto calculate_linear_coefficient_from_limits = [](const auto& limits_for_axes, const auto& norm_vect) -> double
{
    double average_max_accel = 0;
    double average_max_accel_sum = 0;
    for (unsigned int i = 0; i < limits_for_axes.size(); i++) {
        average_max_accel += limits_for_axes.at(i) * std::abs(norm_vect.at(i));
        average_max_accel_sum += std::abs(norm_vect.at(i));
    }
    average_max_accel = average_max_accel / average_max_accel_sum;
    return average_max_accel;
};


template<class T,std::size_t N>
inline generic_position_t<T,N> interpolate_points(const generic_position_t<T,N> &point0,const generic_position_t<T,N> &point1, const double t) { 
    return point0 * (1.0 - t) + point1*t;
};


template<class T,std::size_t N>
inline generic_position_t<T,N> bezier_rec(const std::vector<generic_position_t<T,N>> &points, const double t, const int r, const int i) { 
    // Casteljau algorithm
    if(r == 0) return points[i];
    return (bezier_rec(points, t, r - 1, i) * (1 - t))  + (bezier_rec(points, t, r - 1, i + 1)*t);
};

template<class T,std::size_t N>
inline generic_position_t<T,N> bezier_rec_a(const std::vector<generic_position_t<T,N>> &points, const double t, const int r, const int i) { 
    // Casteljau algorithm
    if(r == 0) return points[i];
    return (bezier_rec_a(points, t, r - 1, i) * (1 - t))  + (bezier_rec_a(points, t, r - 1, i + 1)*t);
};

template<class T,std::size_t N>
inline generic_position_t<T,N> bezier(const std::vector<generic_position_t<T,N>> &points, const double t) {
    if (points.size() == 1) return points[0];
    return bezier_rec_a(points, t, points.size()-1, 0);
}


using distance_t = generic_position_t<double,4>;
using distance_with_velocity_t = generic_position_t<double,5>;

inline distance_t to_distance_t(const distance_with_velocity_t &v) {
    distance_t ret;
    for (size_t i = 0; i < ret.size(); i++) ret[i] = v[i];
    return ret;
};
inline distance_with_velocity_t to_distance_with_v_t (const distance_t &v) {
    distance_with_velocity_t ret;
    for (size_t i = 0; i < v.size(); i++) ret[i] = v[i];
    return ret;
};



template<class T,std::size_t N>
inline double point_segment_distance_3d(const generic_position_t<T,N>& A, const generic_position_t<T,N>& B, const generic_position_t<T,N>& C)
{
    double l = (C - B).length();
    if (l <= 0)
        return (A - B).length();
    auto d = (C - B) / l;
    auto v = A - B;
    double t = v.dot_product(d);
    auto P = B + (d * t);

    return (P - A).length();
}


//distance_t bezier(const std::vector<distance_t> &p, const double t);

/**
 * follows the path where first coordinates are position, and last coordinate is velocity.
 * It will execute on_point for each next position with given velocity and dt
 * @param path_points_with_velocity poins of path with velocity
 * @param on_point callback function that will be executed for each point
 * @param dt - time frame
 * @param min_velocity - minimal accepted velocity. Note that velocity of 0 will result in infinite loop
 * */
template <std::size_t N>
void follow_path_with_velocity(
    const std::vector<generic_position_t<double, N>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, N>& position)> on_point,
    double dt,
    const double min_velocity = 0.025
);

/**
 * @brief @untested
 * @brief calculates bezier spline based on standard path. It tries to 
 * 
 */
template<std::size_t N>
void beizer_spline(const std::vector<generic_position_t<double,N>> &path,
                   std::function<void(const generic_position_t<double,N> &position)> on_point,
                   const double dt, const double arc_l = 1.0, const bool velocity_included = true);



template <class T>
std::vector<char> optimize_generic_path_dp(double epsilon, const std::vector<T>& path);

template <class T>
std::vector <T> optimize_path_dp(std::vector <T> &path, double epsilon);

//// implementations




template <class T, std::size_t N>
inline  T generic_position_t<T, N>::sumv() const
{
    return std::accumulate(this->begin(), this->end(), 0.0);
}

template <std::size_t N>
inline void follow_path_with_velocity(
    const std::vector<generic_position_t<double, N>> &path_points_with_velocity,
    std::function<void(const generic_position_t<double, N>& position)> on_point,
    const double dt,
    const double min_velocity
) {
    const double absolute_min_velocity = 0.000001;
    if (min_velocity < absolute_min_velocity) throw std::invalid_argument("follow_path_with_velocity: min_velocity too small! Must be above " + std::to_string(absolute_min_velocity));
    int consecutive_errors_count = 0;
    if (path_points_with_velocity.size() > 0) {
        auto pos = path_points_with_velocity.front();
        double curr_dist = 0.0;
        auto current_velocity = pos.back(); 
        generic_position_t<double, N> ndistv;
        for (unsigned i = 0; i < path_points_with_velocity.size();) {
            if (current_velocity <= min_velocity) {
                if (consecutive_errors_count > 1) std::cerr << "beizer_spline: velocity too small [" << i << "] " << pos << std::endl;
                current_velocity = min_velocity;
                consecutive_errors_count++;
            } else {
                consecutive_errors_count = 0;
            }
            double target_dist = current_velocity * dt; // s = v * t
            ndistv = path_points_with_velocity[i] - pos; // distance to go between points
            ndistv.back() = 0;
            double ndist = ndistv.length();
            if ((curr_dist + ndist) >= target_dist) {
                auto mv = (ndistv / ndist) * (target_dist - curr_dist);
                pos = pos + mv;
                
                generic_position_t<double, N-1>  segment_vect = path_points_with_velocity[i] - path_points_with_velocity[i-1];
                auto S = segment_vect.length();
                generic_position_t<double, N-1>  dist_to_first = pos - path_points_with_velocity[i-1];
                // generic_position_t<double, N-1>  dist_to_second = path_points_with_velocity[i] - pos;

                if (S <= 0.0) throw std::invalid_argument("follow_path_with_velocity: impossible to calculate for two points in the same place");
                if ((path_points_with_velocity[i].back() < min_velocity) && (path_points_with_velocity[i-1].back() < min_velocity)) throw std::invalid_argument("follow_path_with_velocity: path containing 2 consecutive nodes with 0 velocity and non zero length");

                const double v_0 = path_points_with_velocity[i-1].back();
                const double v_T = path_points_with_velocity[i].back();
                const double a = (v_T*v_T-v_0*v_0 ) / (2.0*S); // acceleration (Tested, it is ok)
                
                const double s = dist_to_first.length();

                
                if (std::abs(a) < 0.00000001) {
                    // v_0*t-s=0
                    pos.back() = v_0;
                } else {
                    // (a/2)*t*t+v_0*t-s=0
                    double sqrt_delta = std::sqrt(  v_0*v_0 - 4*(a/2.0)*(-s)   );
                    // double t_1 = (-v_0 - sqrt_delta)/(2*0.5*a);
                    const double t_2 = (-v_0 + sqrt_delta)/(2*0.5*a);
                    pos.back() = v_0+a*t_2;
                }
                current_velocity = pos.back();
                on_point(pos);
                curr_dist = 0.0;
            } else {
                curr_dist += ndist;
                current_velocity = pos.back();
                pos = path_points_with_velocity[i];
                i++;
            }
        }
        //        on_point(pos);
    }
}


template <std::size_t N>
inline void beizer_spline(const std::vector<generic_position_t<double, N>>& path,
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
            t += dt_p;
        }
        t = t - 1.0;
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
inline void optimize_generic_path_dp_inner(double epsilon, int start, int end, const std::vector<T>& path, std::vector<char>& to_delete)
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
inline std::vector<char> optimize_generic_path_dp(double epsilon, const std::vector<T>& path) {
  std::vector<char> to_delete(path.size(), false);
  //DouglasPeucker algorithm
  optimize_generic_path_dp_inner(epsilon, 0, path.size() - 1, path, to_delete);
  return to_delete;
}

template <class T>
inline std::vector<T> optimize_path_dp(std::vector<T>& path, double epsilon)
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


} // namespace raspigcd
#endif
