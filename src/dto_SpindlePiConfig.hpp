#ifndef __TP_MOTOR_DTO_SPINDLEPICONFIG_HPP__
#define __TP_MOTOR_DTO_SPINDLEPICONFIG_HPP__

#include <nlohmann/json.hpp>

namespace tp {
namespace motor {
    struct SpindlePiConfig {
        unsigned int pin;
        int servopwm;
    };

    void to_json(nlohmann::json& j, const SpindlePiConfig& p);
    void from_json(const nlohmann::json& j, SpindlePiConfig& p);

} // namespace motor
} // namespace tp
#endif
