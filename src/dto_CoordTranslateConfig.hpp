#ifndef __TP_COORD_COORD_TRANSLATE_CONFIG_HPP__
#define __TP_COORD_COORD_TRANSLATE_CONFIG_HPP__

#include "dto_CoordTranslateConfig.hpp"
#include "dto_Steps.hpp"
#include <tpcommon/position.hpp>

#include <nlohmann/json.hpp>

#include <array>

namespace tp {
namespace coord {

    using tp::motor::Steps;

    struct CoordTranslateConfig {
        std::string motorConfiguration; //: "corexy",
        struct {
            double x; //: -1.0,
            double y; //: 1.0,
            double z; //: 1.0
            double t; //: 1.0
        } scale;
        struct {
            double a; //: 82.0512820513,
            double b; //: 82.0512820513,
            double c; //: 200
            double d; //: 100
        } stepsPerMm;
    };

    CoordTranslateConfig getDefaults_CoordTranslateConfig();
    void to_json(nlohmann::json& j, const CoordTranslateConfig& p);
    void from_json(const nlohmann::json& j, CoordTranslateConfig& p);
    std::ostream& operator<<(std::ostream& os, CoordTranslateConfig const& value);
    std::istream& operator>>(std::istream& is, CoordTranslateConfig& value);
    bool operator==(const CoordTranslateConfig& l, const CoordTranslateConfig& r);

} // namespace coord
} // namespace tp
#endif
