/*#include "mode.h"
#include "Plane.h"

void ModeManualK::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);

    plane.nav_roll_cd = plane.ahrs.roll_sensor;
    plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
}
*/

#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if HAL_WITH_IO_MCU || CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>

#include "mode.h"
#include "Plane.h"
#include "zef_control.h"

using namespace AP;

//static AP_Baro barometer;
// Serial manager is needed for UART communications
static AP_SerialManager serial_manager;

//static uint16_t pwm = 1500;
//static int8_t delta = 1;

bool ModeManualK::_enter()
{
    rc().channel(CH_5)->set_range(1900);
    rc().channel(CH_6)->set_range(1900);
    
    //rc().channel(CH_7)->set_range(1900);
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, plane.get_throttle_input(true));
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, plane.get_throttle_input(true));
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, plane.get_throttle_input(true));
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, plane.get_throttle_input(true));
    
    if (!observer) {
        observer = zefObserver::getInstance(plane.ahrs);
    }
    if (!zefiroControl) {
        zefiroControl = ZefControl::getInstance();
    }
    if (!actuador) {
        actuador = zefActuador::getInstance();
    }

    observer->_baro_reference_set = false;

    last_print_ms = 0;
    
    gps().init(zefiroControl->serial_manager);
    gps().update();

    //actuador->set_motor_limits(1000, 2000);
    zefiroControl->set_manual(1);
    
    return true;
}

void ModeManualK::_exit()
{

}

void ModeManualK::update()
{

    observer->update();
    
    zefObserver::AircraftState state = observer->get_state();

    //manual controls
    if(1) {
        double control_aileron = rc().channel(CH_1)->get_control_in();//plane.roll_in_expo(false);
        double control_pitch = rc().channel(CH_2)->get_control_in();//plane.pitch_in_expo(false);
        double control_yaw = rc().channel(CH_4)->get_control_in();//plane.rudder_in_expo(false);
        double control_throttle = rc().channel(CH_3)->get_control_in();//plane.get_throttle_input(true);
        double control_right_switch = rc().channel(CH_5)->get_control_in();
        double control_left_switch = rc().channel(CH_6)->get_control_in();
        
        int normalizador_inputs_apy = 4500;
        int normalizador_throttle = 50;
        int zero_throttle = 50;
        int normalizador_aux = 1900/2;
        int zero_aux = 1900/2;
        control_aileron /= normalizador_inputs_apy;
        control_pitch /= normalizador_inputs_apy;
        control_yaw /= normalizador_inputs_apy;
        control_throttle = (control_throttle - zero_throttle) / normalizador_throttle; //TODO
        control_left_switch = (control_left_switch - zero_aux) / normalizador_aux;
        control_right_switch = (control_right_switch - zero_aux) / normalizador_aux;
        
        double dead_zone = 0.05;
        control_aileron = zefiroControl->dead_zone(control_aileron, dead_zone);
        control_pitch = zefiroControl->dead_zone(control_pitch, dead_zone);
        control_yaw = zefiroControl->dead_zone(control_yaw, dead_zone);
        control_throttle = zefiroControl->dead_zone(control_throttle, dead_zone);
        control_left_switch = zefiroControl->dead_zone(control_left_switch, dead_zone);
        control_right_switch = zefiroControl->dead_zone(control_right_switch, dead_zone);
        
       /* GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ail: %f", control_aileron);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pit: %f", control_pitch);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "yaw: %f", control_yaw);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "trh: %f", control_throttle);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RS: %f", control_right_switch);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LS: %f", control_left_switch);*/
        
        zefiroControl->manual_inputs_update(control_aileron, control_pitch, control_yaw, control_throttle, control_right_switch, control_left_switch);
    }
    //end manual control

    // Define variables for second update using observer data
    double U = state.vx;
    double V = state.vy;
    double W = state.vz;
    double P = state.angular_roll;
    double Q = state.angular_pitch;
    double R = state.angular_yaw;
    float roll = state.roll;
    float pitch = state.pitch;
    float yaw = state.yaw;
    float X = state.X;
    float Y = state.Y;
    float Z = state.Z;

    zefiroControl->update(U, V, W,
                        P, Q, R,
                        roll, pitch, yaw,
                        X, Y, Z);

    double force_vector[12];
    zefiroControl->get_force_vector(force_vector);

    actuador->update(force_vector);
    
}


