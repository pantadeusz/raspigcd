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


#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL_BUTTONS_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL_BUTTONS_T_HPP__

#include <functional>
#include <vector>

namespace raspigcd {
namespace hardware {


class low_buttons
{
private:
public:
    /**
     * @brief attach callback to button down. It will throw exception for not supported button
     * @param callback_ the callback function that will receive button number and new status (0 off, 1 on)
     */
    virtual void on_key(int btn, std::function<void(int,int)> callback_) = 0;

    /**
     * @brief returns current handler for key down
     */
    virtual std::function<void(int,int)> on_key(int btn) = 0;

    /**
     * @brief returns the key state
     */
    virtual std::vector < int > keys_state() = 0;
};

} // namespace hardware
} // namespace raspigcd

#endif
