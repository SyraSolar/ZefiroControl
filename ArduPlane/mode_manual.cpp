#include "mode.h"
#include "Plane.h"

void ModeManual::update()
{
    //double calculo = 9.0 + 3.0 * 3.0;
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Resultado: %f", calculo);
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);

    plane.nav_roll_cd = plane.ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
}

