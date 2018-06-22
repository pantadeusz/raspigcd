#include "i_CoordTranslate.hpp"
#include "i_CoordTranslate_factory.hpp"

#include "gcd_engine.hpp"

#include "tdd_helpers.hpp"
#include "tpcommon/img_img8.hpp"

#include <catch.hpp>
#include <fakeit.hpp>
#include <fstream>
#include <future>
#include <vector>

using namespace fakeit;
using namespace tp::helpers;
using namespace tp::motor;
using namespace tp::coord;
using namespace tp::gcd;
using namespace tp::img;

SCENARIO("Test for configuration conversion between json and cpp object", "[to_json][from_json][GcodeEngineConfig]")
{
    SECTION("Simple config creation and conversion")
    {
        GcodeEngineConfig cfg0 = getDefaults_GcodeEngineConfig();
        nlohmann::json cfg0json = cfg0;
        GcodeEngineConfig cfg1 = cfg0json;
        nlohmann::json cfg1json = cfg1;
        REQUIRE(cfg0json == cfg1json);
        REQUIRE(cfg0 == cfg1);
    }
    SECTION("Are default values preserved if data is incomplete")
    {
        GcodeEngineConfig cfg0 = getDefaults_GcodeEngineConfig();
        nlohmann::json cfg1json;
        cfg1json["config"]["StepperPiConfig"]["m0"]["step"] = 1211;
        GcodeEngineConfig cfg1 = cfg1json;
        cfg0.stepper[0].step = 1211;

        REQUIRE(cfg0 == cfg1);
    }
}

SCENARIO("Test for GcodeEngine class", "[gcd][GcodeEngine]")
{
    Img8 img(1024, 1024);
    Img8 img_dt(1024, 1024);
    Mock<i_Stepper> stepperMock;
    Mock<i_Spindle> spindleMock;
    Mock<i_Buttons> buttonsMock;

    MachineSimulationState mState;
    mState.ctranslate = CoordTranslate_corexy_factory({ 10, 10, 10, 10 });

    auto prevTime = std::chrono::high_resolution_clock::now();
    std::shared_ptr<i_MotorMoves> p_motor = MotorMoves_factory(&stepperMock.get(), &spindleMock.get(), &buttonsMock.get(), 50);
        GcodeEngineConfig gcdwconfiguration = nlohmann::json::parse(R"({
		"config":{
        "CoordTranslate" : {
            "motorConfiguration" : "corexy",
            "stepsPerMm" : {
                "a" : 10,
                "b" : 10,
                "c" : 10
            }
        },
                           "MotorMoves" : {
                               "tickTime" : 100
                           },
                                          "GcdCommandsInterpreter":
        {
            "g0speed" : 60,
                "g1speed" : 20
        }
		}
})" );
	GcodeEngine gcdw ( p_motor, gcdwconfiguration );

GIVEN("the coordinate translation and hardware motors are given")
{

    When(Method(stepperMock, enabled)).AlwaysReturn();
    When(Method(stepperMock, step)).AlwaysDo([&](std::array<signed char, 4>& steps) {
        mState.doStep(steps, true);
    });
    When(Method(spindleMock, setSpeed)).AlwaysReturn();

    WHEN("series of commands are sent via execCommand")
    {
        std::string commands;
        commands = commands + "M17" + "\n";
        commands = commands + "M3" + "\n";
        commands = commands + "G0X5Y10" + "\n";
        commands = commands + "G0X10" + "\n";
        commands = commands + "G0Y-10" + "\n";
        commands = commands + "G4P200" + "\n";
        commands = commands + "G0X-10Y-10" + "\n";
        commands = commands + "G0X-10Y10" + "\n";
        commands = commands + "M5" + "\n";
        commands = commands + "M18" + "\n";
        gcdw.execGcodeProgram(commands, [](int l, const std::string& c, const std::string& r) {
				std::stringstream s;
				s << "(" << l << ") " << c << " -> " << r;
				INFO( s.str() ); }, [](int status) {
				std::stringstream s;
				s << "gcode finished with status " << status;
				INFO( s.str() ); });

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

            Img8 test1("tests/unit_machine_2.png");
            if (!(test1 == img)) {
                (test1 - img).save("build/unit_gcd_engine_1_diff1.png");
                (img - test1).save("build/unit_gcd_engine_1_diff2.png");
                img.save("build/unit_gcd_engine_1.png");
            }
            img_dt.save("build/unit_gcd_engine_1_v_euclidean.png");
            REQUIRE(test1 == img);

            REQUIRE(mState.getPosition()[0] == Approx(-10).epsilon(0.5));
            REQUIRE(mState.getPosition()[1] == Approx(10).epsilon(0.5));
            REQUIRE(mState.getPosition()[2] == Approx(0).epsilon(0.5));
            Verify(Method(stepperMock, enabled).Using(true)).Exactly(1);
            Verify(Method(stepperMock, enabled).Using(false)).Exactly(1);
            Verify(Method(spindleMock, setSpeed).Using(1.0)).Exactly(1);
            Verify(Method(spindleMock, setSpeed).Using(0.0)).Exactly(1);
        }
    }

    WHEN("gcode is loaded from file via ifstream")
    {
        std::ifstream fsgcode("tests/unit_gcd_engine_gcdtest.gcd");
        gcdw.execGcodeProgram(fsgcode, [](int l, const std::string& c, const std::string& r) {
				std::stringstream s;
				s << "(" << l << ") " << c << " -> " << r;
				INFO( s.str() ); }, [](int status) {
				std::stringstream s;
				s << "gcode finished with status " << status;
				INFO( s.str() ); });

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
                if (p[2] <= 0)
                    img(p[0], p[1]) = (int)(p[2] * 64.0) % 192;
                img_dt(p[0], p[1]) = (unsigned char)((dtms >> 4) & 0x0ff);
                ++timesI;
                ++positionsI;
            }

            Img8 test1("tests/unit_gcd_engine_gcdtest.png");
            if (!(test1 == img)) {
                (test1 - img).save("build/unit_gcd_engine2_gcdtest_diff1.png");
                (img - test1).save("build/unit_gcd_engine2_gcdtest_diff2.png");
                img.save("build/unit_gcd_2_engine_gcdtest.png");
            }
            img_dt.save("build/unit_gcd_2_engine_gcdtest_dt.png");
            REQUIRE(test1 == img);

            REQUIRE(mState.getPosition()[0] == Approx(0).epsilon(0.5));
            REQUIRE(mState.getPosition()[1] == Approx(0).epsilon(0.5));
            REQUIRE(mState.getPosition()[2] == Approx(0).epsilon(0.5));
            Verify(Method(stepperMock, enabled).Using(true)).Exactly(1);
            Verify(Method(stepperMock, enabled).Using(false)).Exactly(1);
            Verify(Method(spindleMock, setSpeed).Using(1.0)).Exactly(1);
            Verify(Method(spindleMock, setSpeed).Using(0.0)).Exactly(1);
        }
    }

    WHEN("gcode is loaded from file via ifstream and the execution is terminated")
    {
        std::ifstream fsgcode("tests/unit_gcd_engine_gcdtest.gcd");
        std::future<int> futureResp = std::async(std::launch::async, [&]() {
            return gcdw.execGcodeProgram(fsgcode, [](int l, const std::string& c, const std::string& r) {
					std::stringstream s;
					s << "(" << l << ") " << c << " -> " << r;
					INFO( s.str() ); }, [](int status) {
					std::stringstream s;
					s << "gcode finished with status " << status;
					INFO( s.str() ); });
        });

        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        gcdw.breakExecution();

        THEN("the program should break and the status code should be -2")
        {
            futureResp.wait();
            auto timesI = mState.recordedTimes.begin();
            auto positionsI = mState.recordedPositions.begin();

            while (timesI != mState.recordedTimes.end() && positionsI != mState.recordedPositions.end()) {
                auto dt = prevTime - *timesI;
                prevTime = *timesI;
                int dtms = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
                Position p = mState.getPosition(*positionsI);
                p = p * 10 + Position(img.width / 2, img.height / 2, 0); // 10points per mm
                if (p[2] <= 0)
                    img(p[0], p[1]) = (int)(p[2] * 64.0) % 192;
                img_dt(p[0], p[1]) = (unsigned char)((dtms >> 4) & 0x0ff);
                ++timesI;
                ++positionsI;
            }

            Img8 test1("tests/unit_gcd_engine_gcdtest.png");
            REQUIRE(futureResp.get() == -2);
            REQUIRE(!(test1 == img));
        }
    }

    WHEN("the command to dwell is sent")
    {
        std::string commands;
        commands = commands + "G4P1000" + "\n";
        auto startTime = std::chrono::high_resolution_clock::now();
        gcdw.execGcodeProgram(commands, [&](int l, const std::string& c, const std::string& r) {
				std::stringstream s;
				s << "(" << l << ") " << c << " -> " << r;
				INFO( s.str() ); }, [](int status) {
				std::stringstream s;
				s << "gcode finished with status " << status;
				INFO( s.str() ); });

        auto endTime = std::chrono::high_resolution_clock::now();

        THEN("the dwell command should take exact time")
        {

            int dtms = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            REQUIRE(dtms == Approx(1000).epsilon(0.01));
        }
    }
}
}

TEST_CASE("GcodeEngine generating correct defaults", "[defaults][GcodeEngine]")
{
    GcodeEngineConfig defaults = getDefaults_GcodeEngineConfig();
    nlohmann::json defaultsJson = defaults;
    std::string configurationJsonStr = defaultsJson.dump();
    std::string configurationJsonStrFF;
    std::ifstream configFile("defaults.json");
    if (configFile.is_open()) {
        configurationJsonStrFF = std::string((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());
    }
    GcodeEngineConfig defaultsFromFile = nlohmann::json::parse(configurationJsonStrFF);
    nlohmann::json defaultsFromFileJson = defaultsFromFile;
    configurationJsonStrFF = defaultsFromFileJson.dump();
    REQUIRE(configurationJsonStrFF == configurationJsonStr);
}
