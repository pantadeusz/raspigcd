#include "i_CoordTranslate.hpp"
#include "i_CoordTranslate_factory.hpp"
#include "s_GcdCommandsInterpreter.hpp"

#include "tdd_helpers.hpp"
#include "tpcommon/img_img8.hpp"

#include <catch.hpp>
#include <fakeit.hpp>

#include <iostream>
#include <vector>

using namespace fakeit;
using namespace tp::helpers;
using namespace tp::motor;
using namespace tp::coord;
using namespace tp::gcd;
using namespace tp::img;

SCENARIO("Test for Gcd interpreter class class", "[gcd][GcdCommandsInterpreter]")
{
    Img8 img(1024, 1024);
    Img8 img_dt(1024, 1024);
    Mock<i_Stepper> stepperMock;
    Mock<i_Spindle> spindleMock;
    Mock<i_Buttons> buttonsMock;

    MachineSimulationState mState;
    mState.ctranslate = CoordTranslate_simple_factory({ 10, 10, 10, 10 });

    auto prevTime = std::chrono::high_resolution_clock::now();
    std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 50);
    GcdCommandsInterpreter gcdw;

    GIVEN("the coordinate translation and hardware motors are given")
    {

        When(Method(stepperMock, enabled)).AlwaysReturn();
        When(Method(stepperMock, step)).AlwaysDo([&](std::array<signed char, 4>& steps) {
            mState.doStep(steps, true);
        });
        When(Method(spindleMock, setSpeed)).AlwaysReturn();

        WHEN("series of commands are sent without callbacks")
        {

            auto pp_machine = new Machine();
            std::shared_ptr<i_Machine> p_machine(pp_machine);
            pp_machine->setCoordinateSystem(mState.ctranslate);
            pp_machine->setMotorMoves(p_motor);
            gcdw.setMachine(p_machine);

            gcdw.execCommand("M17");
            gcdw.execCommand("M3");
            gcdw.execCommand("G0X5Y10");
            gcdw.execCommand("G0X10Y10");
            gcdw.execCommand("G0X10Y-10");
            gcdw.execCommand("G0X-10Y-10");
            gcdw.execCommand("G0X-10Y10");
            gcdw.execCommand("M5");
            gcdw.execCommand("M18");
            gcdw.finish();
            THEN("the path followed by machine should match the intended pattern")
            {
                auto timesI = mState.recordedTimes.begin();
                auto positionsI = mState.recordedPositions.begin();

                while (timesI != mState.recordedTimes.end() && positionsI != mState.recordedPositions.end()) {
                    auto dt = prevTime - *timesI;
                    prevTime = *timesI;
                    int dtms = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
                    Position p = mState.getPosition(*positionsI);
                    p = p * 10 + Position(img.width / 2, img.height / 2, 0); // 10points per mm
                    img(p[0], p[1]) = (int)(p[2] * 64.0) % 192;
                    img_dt(p[0], p[1]) = (unsigned char)((dtms >> 4) & 0x0ff);
                    ++timesI;
                    ++positionsI;
                }

                Img8 test1("tests/unit_machine_1.png");
                if (!(test1 == img)) {
                    (test1 - img).save("build/gcd_commands_1_diff1.png");
                    (img - test1).save("build/gcd_commands_1_diff2.png");
                    img.save("build/gcd_commands_1.png");
                }
                img_dt.save("build/gcd_commands_1_v_euclidean.png");
                REQUIRE(test1 == img);

                REQUIRE(mState.getPosition()[0] == Approx(-10).epsilon(0.01));
                REQUIRE(mState.getPosition()[1] == Approx(10).epsilon(0.01));
                REQUIRE(mState.getPosition()[2] == Approx(0).epsilon(0.01));
                Verify(Method(stepperMock, enabled).Using(true)).Exactly(1);
                Verify(Method(stepperMock, enabled).Using(false)).Exactly(1);
                Verify(Method(spindleMock, setSpeed).Using(1.0)).Exactly(1);
                Verify(Method(spindleMock, setSpeed).Using(0.0)).Exactly(1);
            }
        }
    }
}
