/*
    Raspberry Pi G-CODE interpreter

    Copyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl

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





#include <hardware/motor_layout.hpp>

namespace raspigcd {
namespace hardware {

class corexy_layout_t : public motor_layout
{
public:
    std::array<double, 4> scales_; // scale along given axis
    std::array<double, 4> steps_per_milimeter_;

    steps_t cartesian_to_steps(const distance_t& distances_);
    distance_t steps_to_cartesian(const steps_t& steps_);
    void set_configuration(const configuration::actuators_organization& cfg);
};

steps_t corexy_layout_t::cartesian_to_steps(const distance_t& distances_)
{
    return {
        (int)((distances_[0] * scales_[0] + distances_[1] * scales_[1]) * steps_per_milimeter_[0]),
        (int)((distances_[0] * scales_[0] - distances_[1] * scales_[1]) * steps_per_milimeter_[1]),
        (int)(distances_[2] * steps_per_milimeter_[2] * scales_[2])};
}

distance_t corexy_layout_t::steps_to_cartesian(const steps_t& steps_)
{
    return {
        0.5 * (double)(steps_[0] / steps_per_milimeter_[0] + steps_[1] / steps_per_milimeter_[1]) / scales_[0],
        0.5 * (double)(steps_[0] / steps_per_milimeter_[0] - steps_[1] / steps_per_milimeter_[1]) / scales_[1],
        steps_[2] / (steps_per_milimeter_[2] * scales_[2])};
}

void corexy_layout_t::set_configuration(const configuration::actuators_organization& cfg)
{
    for (unsigned int i = 0; i < cfg.steppers.size(); i++) {
        steps_per_milimeter_[i] = cfg.steppers.at(i).steps_per_mm;
        scales_[i] = cfg.scale[i];
    }
}


class cartesian_layout_t : public motor_layout
{
public:
    std::array<double, 4> scales_; // scale along given axis
    std::array<double, 4> steps_per_milimeter_;

    steps_t cartesian_to_steps(const distance_t& distances_);
    distance_t steps_to_cartesian(const steps_t& steps_);
    void set_configuration(const configuration::actuators_organization& cfg);
};

steps_t cartesian_layout_t::cartesian_to_steps(const distance_t& distances_)
{
    steps_t ret;
    for (std::size_t i = 0; i < distances_.size(); i++) ret[i] = distances_[i] * steps_per_milimeter_[i] * scales_[i];
    return ret;
}

distance_t cartesian_layout_t::steps_to_cartesian(const steps_t& steps_)
{
    distance_t ret;
    for (std::size_t i = 0; i < steps_.size(); i++) ret[i] = steps_[i] / (steps_per_milimeter_[i] * scales_[i]);
    return ret;
}


void cartesian_layout_t::set_configuration(const configuration::actuators_organization& cfg)
{
    for (unsigned int i = 0; i < cfg.steppers.size(); i++) {
        steps_per_milimeter_[i] = cfg.steppers.at(i).steps_per_mm;
        scales_[i] = cfg.scale[i];
    }
}


std::shared_ptr<motor_layout> motor_layout::get_instance(const configuration::actuators_organization& cfg)
{
    if (cfg.motion_layout == configuration::motion_layouts::COREXY) {
        std::shared_ptr<motor_layout> ret(new corexy_layout_t());
        ret.get()->set_configuration(cfg);
        return ret;
    } else if (cfg.motion_layout == configuration::motion_layouts::CARTESIAN) {
        std::shared_ptr<motor_layout> ret(new cartesian_layout_t());
        ret.get()->set_configuration(cfg);
        return ret;
    } else {
        throw std::invalid_argument("Incorrect layout name. Only layouts allowed are: corexy cartesian");
    }
}


} // namespace hardware
} // namespace raspigcd
