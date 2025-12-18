#include "mode.h"
#include "Plane.h"

#include "zef_actuator.h"

bool ModeFBWA::_enter()
{
    if (!actuador) {
        actuador = zefActuador::getInstance();
    }
    return true;
}

void ModeFBWA::update()
{
    actuador->set_motor_servo2origin();
}