#ifndef __TP_GCD_DTO_GCODE_ENGINE_CONFIG_HPP__
#define __TP_GCD_DTO_GCODE_ENGINE_CONFIG_HPP__

#include "gcd_commands.hpp"
#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"
#include "m_motor_rpi.hpp"
#include <istream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace tp {
namespace gcd {

    /**
 * @brief Complete gcode engine configuration. Parts of it goes to different elements configured!
 *
 ************/

    struct GcodeEngineConfig {
        tp::coord::CoordTranslateConfig CoordTranslate;
        struct {
            std::string simulationFileOutput;
            double toolD;
            double dpi;
        } GcodeEngine;
        struct {
            double g0speed; //: 160,
            double g1speed; //: 5,
            double g0speedV0; //:15,
            double g0speedV0ddt; //:1
        } GcdCommandsInterpreter;
        struct {
            double tickTime; //: 100
        } MotorMoves;

        tp::motor::SpindlePiConfig spindle;
        std::vector<tp::motor::StepperPiConfig> stepper;
        tp::motor::ButtonsPiConfig buttons;
    };

    GcodeEngineConfig getDefaults_GcodeEngineConfig();

    void to_json(nlohmann::json& j, const GcodeEngineConfig& p);
    void from_json(const nlohmann::json& j, GcodeEngineConfig& p);
    std::ostream& operator<<(std::ostream& os, GcodeEngineConfig const& value);
    std::istream& operator>>(std::istream& is, GcodeEngineConfig& value);
    bool operator==(const GcodeEngineConfig& l, const GcodeEngineConfig& r);

} // namespace gcd
} // namespace tp

#endif
