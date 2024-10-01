
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Display_tca.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#define TCA9548A_REG_CONFIG    0x70

extern const AP_HAL::HAL& hal;

// constructor
Display_tca_i2c::Display_tca_i2c() :
    _bus(1)
{
    //hw_init();
}

Display_tca_i2c::~Display_tca_i2c()
{
}


Display_tca_i2c *Display_tca_i2c::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    Display_tca_i2c *driver = new Display_tca_i2c(); //std::move(dev));
    if (!driver || !driver->hw_init()) {
        delete driver;
        return nullptr;
    }
    return driver;
}

bool Display_tca_i2c::hw_init()
{
    _dev = std::move(hal.i2c_mgr->get_device(_bus, TCA9548A_REG_CONFIG));
    if (!_dev) {
        return false;
    }
    //WITH_SEMAPHORE(_dev->get_semaphore());

    //_dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    //_dev->write_register(TCA9548A_REG_CONFIG, 1 << 0); //, true);

    _dev->register_periodic_callback(20 * 5, FUNCTOR_BIND_MEMBER(&Display_tca_i2c::_timer, void));

    return true;
}

void Display_tca_i2c::hw_update()
{
    _need_hw_update = true;
}

void Display_tca_i2c::_timer()
{
    if (!_need_hw_update) {
        return;
    }
    _need_hw_update = false;
}

void Display_tca_i2c::set_pixel(uint16_t x, uint16_t y)
{
    if(!_dev) {
        hw_init();
    }
    //write_register(reg.regnum, reg.value);
    _dev->write_register(TCA9548A_REG_CONFIG, 1 << (uint8_t)x);
}

void Display_tca_i2c::clear_pixel(uint16_t x, uint16_t y)
{

}

void Display_tca_i2c::clear_screen()
{

}