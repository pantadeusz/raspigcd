/*

    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include "i_CoordTranslate.hpp"
#include "s_CoordTranslateCoreXY.hpp"
#include "s_CoordTranslateSimple.hpp"
#include "i_CoordTranslate_factory.hpp"

#include <stdexcept>
#include <vector>

namespace tp {
namespace coord {


    std::shared_ptr<i_CoordTranslate> CoordTranslate_corexy_factory(const std::array<double, 4> stepsPerMM_, const Position scaleAxis)
    {
        return std::shared_ptr<i_CoordTranslate>(new CoordTranslateCoreXY(stepsPerMM_, scaleAxis));
    }
    std::shared_ptr<i_CoordTranslate> CoordTranslate_simple_factory(const std::array<double, 4> stepsPerMM_, const Position scaleAxis)
    {
        return std::shared_ptr<i_CoordTranslate>(new CoordTranslateSimple(stepsPerMM_, scaleAxis));
    }

    std::shared_ptr<i_CoordTranslate> CoordTranslate_factory(const CoordTranslateConfig& config)
    {
        if (config.motorConfiguration == "corexy") {
            return CoordTranslate_corexy_factory({ config.stepsPerMm.a, config.stepsPerMm.b, config.stepsPerMm.c, config.stepsPerMm.d },
                { config.scale.x, config.scale.y, config.scale.z, config.scale.t });
        }
        if (config.motorConfiguration == "simple") {
            return CoordTranslate_simple_factory({ config.stepsPerMm.a, config.stepsPerMm.b, config.stepsPerMm.c, config.stepsPerMm.d }, { config.scale.x, config.scale.y, config.scale.z, config.scale.t });
        }
        throw std::domain_error("config.motorConfiguration can be only corexy or simple");
    }

} // namespace coord
} // namespace tp

