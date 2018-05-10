#ifndef __TDD_HELPERS_HPP____
#define __TDD_HELPERS_HPP____

#include <m_motor_interface.hpp>
#include "coordsystem.hpp"
#include <list>
#include <catch.hpp>
#include <fakeit.hpp>

namespace tp {
namespace helpers {

class MachineSimulationState {
public:
    std::list < std::chrono::time_point<std::chrono::high_resolution_clock> > recordedTimes;
    std::list < tp::motor::Steps > recordedPositions;
	std::shared_ptr < tp::coord::i_CoordTranslate > ctranslate;
    std::vector < int > mockSteps;
    
    
    tp::motor::Steps getSteps();
    tp::coord::Position getPosition();
    tp::coord::Position getPosition(const tp::motor::Steps &s);
    void doStep(std::array<signed char, 3> &s, bool recordTimes); // steps callback
    MachineSimulationState() {
        mockSteps = {0,0,0};
    }
};

}
}
#endif
