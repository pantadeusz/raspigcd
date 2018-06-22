#include "i_CoordTranslate.hpp"
#include "i_CoordTranslate_factory.hpp"

#include "i_MotorMoves_factory.hpp"
#include "machine.hpp"

#include <tpcommon/img_img8.hpp>

#include "tdd_helpers.hpp"

#include <catch.hpp>
#include <fakeit.hpp>

#include <iostream>
#include <vector>

using namespace fakeit;
using namespace tp::helpers;
using namespace tp::motor;
using namespace tp::img;
using namespace tp::coord;

TEST_CASE("Test for Machine class", "[machine][Machine]")
{
    Img8 img(1024, 1024);
    Img8 img_dt(1024, 1024);
    std::shared_ptr<tp::coord::i_CoordTranslate> ctranslate = CoordTranslate_simple_factory({ 1, 1, 1, 1 });

    Mock<i_Stepper> stepperMock;
    Mock<i_Spindle> spindleMock;
    Mock<i_Buttons> buttonsMock;
    std::vector<int> mockSteps = { 0, 0, 0 };
    Position mockPosition;
    auto prevTime = std::chrono::high_resolution_clock::now();
    When(Method(stepperMock, enabled)).AlwaysReturn();
    When(Method(stepperMock, step)).AlwaysDo([&](std::array<signed char, 4> steps) {
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

        if (ctranslate) {
            auto nowTime = std::chrono::high_resolution_clock::now();
            Steps st(mockSteps[0], mockSteps[1], mockSteps[2], mockSteps[3]);
            Position p(ctranslate->translate(st));
            mockPosition = p;
            //std::cout << " p: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
            p = p * 10 + Position(img.width / 2, img.height / 2, 0, 0); // 10points per mm
            img(p[0], p[1]) = (int)(p[2] * 64.0) % 192;
            auto dt = prevTime - nowTime;
            prevTime = nowTime;
            int dtms = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
            img_dt(p[0], p[1]) = (unsigned char)((dtms >> 4) & 0x0ff);
        }
    });
    When(Method(spindleMock, setSpeed)).AlwaysReturn();

    SECTION("test standard euclidean coordinate system")
    {

        {
            ctranslate = CoordTranslate_simple_factory({ 10, 10, 10, 10 });
            std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
            Machine machine;

            machine.setCoordinateSystem(ctranslate);
            machine.setMotorMoves(p_motor);

            machine.waitFinish();
            REQUIRE(machine.getPosition() == Position(0, 0, 0, 0));
            machine.gotoXYZ(Position(5, 10, 0), 10);
            machine.gotoXYZ(Position(10, 10, 10), 10, 0x03);
            machine.gotoXYZ(Position(10, -10, 0), 10);
            machine.gotoXYZ(Position(-10, -10, 0), 10);
            machine.gotoXYZ(Position(-10, 10, 0), 10);
            machine.waitFinish();
            ctranslate = 0;
            REQUIRE(machine.getPosition() == Position(-10, 10, 0));
        }

        Img8 test1("tests/unit_machine_1.png");
        if (!(test1 == img)) {
            (test1 - img).save("build/machine_1_diff1.png");
            (img - test1).save("build/machine_1_diff2.png");
            img.save("build/machine_1.png");
        }
        img_dt.save("build/machine_1_v_euclidean.png");
        REQUIRE(test1 == img);

        REQUIRE(mockPosition[0] == Approx(-10).epsilon(0.01));
        REQUIRE(mockPosition[1] == Approx(10).epsilon(0.01));
        REQUIRE(mockPosition[2] == Approx(0).epsilon(0.01));
    }

    SECTION("test core xy coordinate system")
    {

        {
            ctranslate = CoordTranslate_corexy_factory({ 10, 10, 10, 10 });
            std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
            Machine machine;

            machine.setCoordinateSystem(ctranslate);
            machine.setMotorMoves(p_motor);
            REQUIRE(machine.getPosition() == Position(0, 0, 0));

            machine.waitFinish();
            machine.gotoXYZ(Position(5, 10, 0), 10);
            machine.gotoXYZ(Position(10, 10, 0), 10);
            machine.gotoXYZ(Position(10, -10, 0), 10);
            machine.waitFinish();

            REQUIRE(machine.getPosition() == Position(10, -10, 0));
            machine.gotoXYZ(Position(-10, -10, 0), 10);
            machine.gotoXYZ(Position(-10, 10, 0), 10);
            machine.waitFinish();
            REQUIRE(machine.getPosition() == Position(-10, 10, 0));
            ctranslate = 0;
        }

        Img8 test1("tests/unit_machine_2.png");
        if (!(test1 == img)) {
            img.save("build/machine_2.png");
            (test1 - img).save("build/machine_2_diff1.png");
            (img - test1).save("build/machine_2_diff2.png");
        }
        img_dt.save("build/machine_2_v_corexy.png");
        REQUIRE(test1 == img);

        REQUIRE(mockPosition[0] == Approx(-10).epsilon(0.01));
        REQUIRE(mockPosition[1] == Approx(10).epsilon(0.01));
        REQUIRE(mockPosition[2] == Approx(0).epsilon(0.01));
    }
}
