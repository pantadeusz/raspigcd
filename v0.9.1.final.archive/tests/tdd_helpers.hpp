#ifndef __TDD_HELPERS_HPP____
#define __TDD_HELPERS_HPP____

#include "i_CoordTranslate.hpp"
#include <catch.hpp>
#include <fakeit.hpp>
#include <list>
#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"
#include <chrono>


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
