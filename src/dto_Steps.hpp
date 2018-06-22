#ifndef __TP_MOTOR_STEPS_HPP__
#define __TP_MOTOR_STEPS_HPP__

#include <array>

namespace tp {
namespace motor {

    // kroki
    class Steps : public std::array<int, 4> {
    public:
        //Steps(const int a, const int b, const int c) : std::array<int, 4>{a, b, c, 0} {}
        Steps(const int a, const int b, const int c, const int d)
            : std::array<int, 4>{ a, b, c, d }
        {
        }
        Steps()
            : std::array<int, 4>{ 0, 0, 0, 0 }
        {
        }
    };

    inline Steps operator+(const Steps& a, const Steps& b)
    {
        return Steps(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
    }
    inline Steps operator-(const Steps& a, const Steps& b)
    {
        return Steps(a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]);
    }
    inline Steps operator*(const Steps& a, const Steps& b)
    {
        return Steps(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
    }
    inline Steps operator*(const Steps& a, const int& b)
    {
        return Steps(a[0] * b, a[1] * b, a[2] * b, a[3] * b);
    }
    inline Steps operator/(const Steps& a, const int& b)
    {
        return Steps(a[0] / b, a[1] / b, a[2] / b, a[3] / b);
    }
    inline Steps operator/(const Steps& a, const Steps& b)
    {
        return Steps(a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]);
    }
    inline double len2(Steps& s)
    {
        return s[0] * s[0] + s[1] * s[1] + s[2] * s[2] + s[3] * s[3];
    }
    inline double len2(Steps& s, bool x, bool y, bool z, bool t = false)
    {
        return (x ? s[0] * s[0] : 0) + (y ? s[1] * s[1] : 0) + (z ? s[2] * s[2] : 0) + (t ? s[3] * s[3] : 0);
    }

} // namespace motor
} // namespace tp

#endif