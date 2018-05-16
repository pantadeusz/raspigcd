#ifndef __TDD_HELPERS_HPP____
#define __TDD_HELPERS_HPP____

#include "coordsystem.hpp"
#include <catch.hpp>
#include <fakeit.hpp>
#include <list>
#include <m_motor_interface.hpp>

namespace tp {
namespace helpers {

    class MachineSimulationState {
    public:
        std::list<std::chrono::time_point<std::chrono::high_resolution_clock> > recordedTimes;
        std::list<tp::motor::Steps> recordedPositions;
        std::shared_ptr<tp::coord::i_CoordTranslate> ctranslate;
        std::vector<int> mockSteps;

        tp::motor::Steps getSteps();
        tp::coord::Position getPosition();
        tp::coord::Position getPosition(const tp::motor::Steps& s);
        void doStep(std::array<signed char, 4>& s, bool recordTimes); // steps callback
        MachineSimulationState()
        {
            mockSteps = { 0, 0, 0 };
        }
    };

} // namespace helpers
} // namespace tp
#endif
