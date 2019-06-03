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



#ifndef __RASPIGCD_generic_position_t_HPP__
#define __RASPIGCD_generic_position_t_HPP__

#include <array>
#include <vector>
#include <iostream>
#include <cassert>
#include <cmath>
#include <functional>

#include <hardware_dof_conf.hpp>

namespace raspigcd {
//RASPIGCD_HARDWARE_DOF
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
    auto u = a - (*this);
    auto v = b - (*this);
    auto dotprod = (u * v).sumv();
    if (dotprod == 0) return 3.14159265358979323846 / 2.0;
    return acos(dotprod / (std::sqrt(u.length2()) * std::sqrt(v.length2())));
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


} // namespace raspigcd
#endif
