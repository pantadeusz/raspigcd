#ifndef __TP_MOTOR_DTO_STEPPERPICONFIG__
#define __TP_MOTOR_DTO_STEPPERPICONFIG__

#include <nlohmann/json.hpp>

namespace tp {
namespace motor {
    struct StepperPiConfig {
        unsigned int step;
        unsigned int dir;
        unsigned int en;
    };
    void to_json(nlohmann::json& j, const StepperPiConfig& p);
    void from_json(const nlohmann::json& j, StepperPiConfig& p);
    inline bool operator==(const StepperPiConfig& l, const StepperPiConfig& r)
    {
        return (l.step == r.step) && (l.dir == r.dir) && (l.en == r.en);
    };

    inline std::ostream& operator<<(std::ostream& os, StepperPiConfig const& value)
    {
        os << value.step << ", " << value.dir << ", " << value.en;
        return os;
    }
} // namespace motor
} // namespace tp

#endif
