#ifndef __TP_MOTOR_PUTTONSPICONFIG_HPP__
#define __TP_MOTOR_PUTTONSPICONFIG_HPP__
#include <nlohmann/json.hpp>

namespace tp {
namespace motor {

    struct ButtonsPiConfig {
        unsigned int x;
        unsigned int y;
        unsigned int z;
        unsigned int t;
    };

    void to_json(nlohmann::json& j, const ButtonsPiConfig& p);
    void from_json(const nlohmann::json& j, ButtonsPiConfig& p);

} // namespace motor
} // namespace tp

#endif
