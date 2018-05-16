
#include "tdd_helpers.hpp"
#include "coordsystem.hpp"
#include "m_motor_interface.hpp"
#include "tpcommon/img_img8.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace fakeit;
using namespace tp::helpers;
using namespace tp::motor;
using namespace tp::img;
using namespace tp::coord;

namespace tp {
namespace helpers {

    tp::motor::Steps MachineSimulationState::getSteps()
    {
        return Steps(mockSteps[0], mockSteps[1], mockSteps[2], mockSteps[3]);
    }

    tp::coord::Position MachineSimulationState::getPosition(const tp::motor::Steps& s)
    {
        return Position(ctranslate->translate(s));
    }

    tp::coord::Position MachineSimulationState::getPosition()
    {
        return Position(ctranslate->translate(getSteps()));
    }
    void MachineSimulationState::doStep(std::array<signed char, 4>& steps, bool recordTimes)
    {
        if (recordTimes)
            recordedTimes.push_back(std::chrono::high_resolution_clock::now());
        for (unsigned i = 0; i < steps.size(); i++) {
            if (steps[i] > 0) {
                steps[i] -= 1;
                mockSteps[i] += 1;
            }
            if (steps[i] < 0) {
                steps[i] += 1;
                mockSteps[i] -= 1;
            }
        }
        if (recordTimes)
            recordedPositions.push_back(getSteps());
    }

} // namespace helpers
} // namespace tp
