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


#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL_BUTTONS_FAKE_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL_BUTTONS_FAKE_T_HPP__

#include <hardware/low_buttons.hpp>

namespace raspigcd {
namespace hardware {
namespace driver {


class low_buttons_fake : public low_buttons {
private:
    std::vector<std::function<void(int,int)> > _key_callbacks;
    std::vector<int> _key_state;
public:
    /**
     * @brief attach callback to button down. It will throw exception for not supported button
     * @param callback_ the callback function that will receive button number and new status
     */
    void on_key(int btn, std::function<void(int,int)> callback_) {
        if ((unsigned)btn < _key_callbacks.size()) {
            _key_callbacks[btn] = callback_;
        }
    };

    /**
     * @brief returns current handler for key down
     */
    std::function<void(int,int)> on_key(int btn) {
        return _key_callbacks.at(btn);
    };
    
    /**
     * @brief returns the key state
     */
    virtual std::vector < int > keys_state(){
        return _key_state;
    };

    void trigger_button_down(int n) {
        trigger_button(n, 1);
    }
    void trigger_button_up(int n) {
        trigger_button(n, 0);
    }
    void trigger_button(int n, int v) {
        _key_state.at(n) = v;
        auto f = _key_callbacks.at(n);
        f(n,_key_state[n]);
    }
    low_buttons_fake(int max_supported_keys) {
        for (int i = 0; i < max_supported_keys; i++) {
            _key_callbacks.push_back([](int,int){});
            _key_state.push_back(0);
        }
    }
};

}
} // namespace hardware
} // namespace raspigcd

#endif
