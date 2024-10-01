#pragma once

#include "Display.h"
#include "Display_Backend.h"
#include <AP_HAL/I2CDevice.h>

#ifndef HAL_MULTI_TCA9548A_I2C_ADDR
 #define HAL_MULTI_TCA9548A_I2C_ADDR  (0x70)
#endif
#ifndef HAL_MULTI_TCA9548A_I2C_ADDR2
 #define HAL_MULTI_TCA9548A_I2C_ADDR2 (0x71)
#endif

class Display_tca_i2c: public Display_Backend {

public:

    static Display_tca_i2c *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    void hw_update() override;
    void set_pixel(uint16_t x, uint16_t y) override;
    void clear_pixel(uint16_t x, uint16_t y) override;
    void clear_screen() override;
    Display_tca_i2c();
    ~Display_tca_i2c() override;

protected:

private:
    uint8_t _bus;
    bool hw_init() override;

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    bool _need_hw_update;
};
