#include "dto_CoordTranslateConfig.hpp"

namespace tp {
namespace coord {

    CoordTranslateConfig getDefaults_CoordTranslateConfig()
    {
        CoordTranslateConfig ret;
        ret.motorConfiguration = "corexy";
        ret.scale.x = 1.0;
        ret.scale.y = 1.0;
        ret.scale.z = 1.0;
        ret.scale.t = 1.0;
        ret.stepsPerMm.a = 100; //82.0512820513;
        ret.stepsPerMm.b = 100; //82.0512820513;
        ret.stepsPerMm.c = 100; //200;
        ret.stepsPerMm.d = 100; //200;
        return ret;
    }

    void to_json(nlohmann::json& j, const CoordTranslateConfig& p)
    {
        j = nlohmann::json{

            { "motorConfiguration", p.motorConfiguration }, //": "corexy",
            {
                "scale", {
                             { "x", p.scale.x }, //: -1.0,
                             { "y", p.scale.y }, //: 1.0,
                             { "z", p.scale.z }, //: 1.0,
                             { "t", p.scale.t } //: 1.0
                         } },
            { "stepsPerMm", {
                                { "a", p.stepsPerMm.a }, //: 82.0512820513,
                                { "b", p.stepsPerMm.b }, //: 82.0512820513,
                                { "c", p.stepsPerMm.c }, //: 200
                                { "d", p.stepsPerMm.d } //: 200
                            } }

        };
    }

    void from_json(const nlohmann::json& j, CoordTranslateConfig& p)
    {
        p = getDefaults_CoordTranslateConfig();
        try {
            p.motorConfiguration = j["motorConfiguration"].get<std::string>();
        } catch (...) {
        }; // corexy
        try {
            p.scale.x = j["scale"]["x"].get<double>();
        } catch (...) {
        }; // -1.0
        try {
            p.scale.y = j["scale"]["y"].get<double>();
        } catch (...) {
        }; // 1.0
        try {
            p.scale.z = j["scale"]["z"].get<double>();
        } catch (...) {
        }; // 1.0
        try {
            p.scale.t = j["scale"]["t"].get<double>();
        } catch (...) {
        }; // 1.0
        try {
            p.stepsPerMm.a = j["stepsPerMm"]["a"].get<double>();
        } catch (...) {
        }; // 82.0512820513
        try {
            p.stepsPerMm.b = j["stepsPerMm"]["b"].get<double>();
        } catch (...) {
        }; // 82.0512820513
        try {
            p.stepsPerMm.c = j["stepsPerMm"]["c"].get<double>();
        } catch (...) {
        }; // 200
        try {
            p.stepsPerMm.d = j["stepsPerMm"]["d"].get<double>();
        } catch (...) {
        }; // 200
    }

    std::ostream& operator<<(std::ostream& os, CoordTranslateConfig const& value)
    {
        nlohmann::json j;
        j = value;
        os << j.dump();
        return os;
    }

    std::istream& operator>>(std::istream& is, CoordTranslateConfig& value)
    {
        nlohmann::json j;
        is >> j;
        value = j;
        return is;
    }

    bool operator==(const CoordTranslateConfig& l, const CoordTranslateConfig& r)
    {
        if (l.motorConfiguration != r.motorConfiguration)
            return false;
        if (l.scale.x != r.scale.x)
            return false;
        if (l.scale.y != r.scale.y)
            return false;
        if (l.scale.z != r.scale.z)
            return false;
        if (l.scale.t != r.scale.t)
            return false;
        if (l.stepsPerMm.a != r.stepsPerMm.a)
            return false;
        if (l.stepsPerMm.b != r.stepsPerMm.b)
            return false;
        if (l.stepsPerMm.c != r.stepsPerMm.c)
            return false;
        if (l.stepsPerMm.d != r.stepsPerMm.d)
            return false;
        return true;
    }
} // namespace coord
} // namespace tp
