#include "i_MotorMoves_factory.hpp"
#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"
#include "m_motor_rpi.hpp"
#include <tpcommon/img_img8.hpp>

#include "tdd_helpers.hpp"

#include <chrono>
#include <list>
#include <thread>

#include <catch.hpp>
#include <fakeit.hpp>

#include <iostream>
#include <vector>

using namespace tp::motor;
using namespace tp::helpers;
using namespace tp::img;

using namespace fakeit;

using std::chrono::seconds; // nanoseconds, system_clock, seconds
using std::this_thread::sleep_for; // sleep_for, sleep_until

TEST_CASE("motor timings", "[timings][MotorMoves]")
{

    int stepsToPerform = 1000;
    int delayPerStep = 250;
    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock> > recordedTimes;
    Mock<i_Stepper> stepperMock;
    Mock<i_Spindle> spindleMock;
    Mock<i_Buttons> buttonsMock;

    recordedTimes.reserve(stepsToPerform + 4);
    When(Method(stepperMock, enabled)).Return();
    When(Method(stepperMock, step)).AlwaysDo([&recordedTimes](std::array<signed char, 4> steps) {
        recordedTimes.push_back(std::chrono::high_resolution_clock::now());
        for (unsigned i = 0; i < steps.size(); i++) {
            if (steps[i] > 0)
                steps[i] -= 1;
            if (steps[i] < 0)
                steps[i] += 1;
        }
    });

    When(Method(spindleMock, setSpeed)).Return();

    auto exectime = std::chrono::high_resolution_clock::now() - std::chrono::high_resolution_clock::now();
    auto startBenchmarkTime = std::chrono::high_resolution_clock::now();
    {
        std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
        i_MotorMoves& motorMoves = *p_motor.get();
        std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() + std::chrono::microseconds(1000000));

        startBenchmarkTime = std::chrono::high_resolution_clock::now();
        for (int j = 0; j < stepsToPerform; j++) {
            MotorCommand cmd = { delayPerStep, { 1, 1, 1 }, MotorCommand::Command::step };
            motorMoves.push(cmd);
        }
        motorMoves.wait_finished();
        exectime = std::chrono::high_resolution_clock::now() - startBenchmarkTime;
    }

    int exectDuration = std::chrono::duration_cast<std::chrono::microseconds>(exectime).count();

    auto prtime = startBenchmarkTime;
    int i = 0;
    for (auto tt : recordedTimes) {
        auto exectime = tt - prtime;
        int exectDuration = std::chrono::duration_cast<std::chrono::microseconds>(exectime).count();
        if (i > 0)
            REQUIRE(exectDuration >= 30);
        prtime = tt;
        i++;
    }
    REQUIRE(exectDuration >= (delayPerStep * stepsToPerform * 99 / 100));
    REQUIRE(exectDuration <= (delayPerStep * stepsToPerform * 115 / 100)); //
    //1000610
    INFO("time of operations: " << exectDuration << " microseconds");

    //	Verify( Method( stepperMock[0],step ).Using( 1 ) + Method( stepperMock[1],step ).Using( 1 ) + Method( stepperMock[2],step ).Using( 1 ) ).Exactly( stepsToPerform );
}

TEST_CASE("motor paths", "[paths][MotorMoves]")
{

    Img8 img(1024, 1024);

    Mock<i_Stepper> stepperMock;
    Mock<i_Spindle> spindleMock;
    Mock<i_Buttons> buttonsMock;

    std::vector<int> position = { (int)img.width / 2, (int)img.height / 2, 0 };

    When(Method(stepperMock, enabled)).AlwaysReturn();
    When(Method(stepperMock, step)).AlwaysDo([&](std::array<signed char, 4>& steps) {
        bool updt = false;
        for (unsigned i = 0; i < steps.size(); i++) {
            //position[i] += steps[i];
            if (steps[i] > 0) {
                steps[i] -= 1;
                position[i] += 1;
                updt = true;
            }
            if (steps[i] < 0) {
                steps[i] += 1;
                position[i] -= 1;
                updt = true;
            }
        }
        if (!updt)
            std::cout << "Called with empty movement" << std::endl;
        img(position[0], position[1]) = position[2] % 256;
    });

    When(Method(spindleMock, setSpeed)).AlwaysReturn();

    SECTION("simple movement always positive")
    {
        {
            std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
            i_MotorMoves& motorMoves = *p_motor.get();
            motorMoves.steps_from_origin({ (int)img.width / 2, (int)img.height / 2, 0, 0 });
            for (int j = 0; j < 10; j++) {
                MotorCommand cmd = { 60, { 1, 1, 1 }, MotorCommand::Command::step };
                motorMoves.push(cmd);
            }
            motorMoves.wait_finished();
        }
        Verify(Method(stepperMock, step)).Exactly(10);
        REQUIRE(position[0] == img.width / 2 + 10);
        REQUIRE(position[1] == img.height / 2 + 10);
        REQUIRE(position[2] == 10);

        img.save("build/unit_motor_1.png");
    }

    SECTION("simple movement always negative")
    {
        Steps stepsDone;
        {
            std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
            i_MotorMoves& motorMoves = *p_motor.get();
            motorMoves.steps_from_origin({ (int)img.width / 2, (int)img.height / 2, 0, 0 });
            for (int j = 0; j < 100; j++) {
                MotorCommand cmd = { 60, { -1, -1, -1 }, MotorCommand::Command::step };
                motorMoves.push(cmd);
            }
            motorMoves.wait_finished();
            stepsDone = motorMoves.steps_from_origin();
        }
        //Verify( Method( stepperMock,step ).Using( -1 ) + Method( stepperMock,step ).Using( -1 ) + Method( stepperMock,step ).Using( -1 ) ).Exactly( 100 );
        REQUIRE(position[0] == img.width / 2 - 100);
        REQUIRE(position[1] == img.height / 2 - 100);
        REQUIRE(position[2] == -100);

        REQUIRE(position[0] == stepsDone[0]);
        REQUIRE(position[1] == stepsDone[1]);
        REQUIRE(position[2] == stepsDone[2]);
        img.save("build/unit_motor_2.png");
    }

    SECTION("motors enable and disable commands")
    {
        Steps stepsDone;
        {
            std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
            i_MotorMoves& motorMoves = *p_motor.get();

            MotorCommand cmd[] = { { 100, { 0, 0, 0 }, MotorCommand::Command::steppersOn }, { 100, { 0, 0, 0 }, MotorCommand::Command::steppersOff } };
            motorMoves.push(cmd[0]);
            motorMoves.push(cmd[1]);
            motorMoves.wait_finished();
        }
        Verify(Method(stepperMock, enabled).Using(true) + Method(stepperMock, enabled).Using(false)).Exactly(1);
    }

    SECTION("spindle enable and disable commands")
    {
        Steps stepsDone;
        {
            std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 100);
            i_MotorMoves& motorMoves = *p_motor.get();

            MotorCommand cmd[] = { { 100, { 0, 0, 0 }, MotorCommand::Command::spindle }, { 100, { 127, 0, 0 }, MotorCommand::Command::spindle }, { 100, { 0, 0, 0 }, MotorCommand::Command::spindle } };
            motorMoves.push(cmd[0]); // off
            motorMoves.push(cmd[1]); // on
            motorMoves.push(cmd[2]); // off
            motorMoves.wait_finished();
        }
        Verify(Method(spindleMock, setSpeed).Using(0) + Method(spindleMock, setSpeed).Using(1) + Method(spindleMock, setSpeed).Using(0)).Exactly(1);
    }
}
