#include "s_CoordTranslateSimple.hpp"
#include <stdexcept>
#include <vector>

namespace tp {
namespace coord {

    CoordTranslateSimple::CoordTranslateSimple(const std::array<double, 4> stepsPerMM_, const Position scaleAxis)
    {
        //if ( stepsPerMm_.size() < 4 ) throw "4 axis must be";
        if ((mX = stepsPerMM_[0]) == 0)
            throw std::invalid_argument("mX must not be 0");
        if ((mY = stepsPerMM_[1]) == 0)
            throw std::invalid_argument("mY must not be 0");
        if ((mZ = stepsPerMM_[2]) == 0)
            throw std::invalid_argument("mZ must not be 0");
        if ((mT = stepsPerMM_[3]) == 0)
            throw std::invalid_argument("mT must not be 0");
        if ((sX = scaleAxis[0]) == 0)
            throw std::invalid_argument("sX must not be 0");
        if ((sY = scaleAxis[1]) == 0)
            throw std::invalid_argument("sY must not be 0");
        if ((sZ = scaleAxis[2]) == 0)
            throw std::invalid_argument("sZ must not be 0");
        if ((sT = scaleAxis[3]) == 0)
            throw std::invalid_argument("sT must not be 0");
    }
    Steps CoordTranslateSimple::translate(const Position& pos)
    {
        return Steps(pos[0] * mX * sX, pos[1] * mY * sY, pos[2] * mZ * sZ, pos[3] * mT * sT);
    };
    Position CoordTranslateSimple::translate(const Steps& steps)
    {
        return Position((double)steps[0] / (sX * mX), (double)steps[1] / (sY * mY), (double)steps[2] / (sZ * mZ), (double)steps[3] / (sT * mT));
    };
} // namespace coord
} // namespace tp
