#include <cstdio> 

#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_InertialNav/AP_InertialNav.h>

#include "mode.h"
#include "Plane.h"

#include "zef_observer.h"
#include "zef_control.h"
#include "zef_actuator.h"

using namespace AP;

bool ModeManualHover::_enter()
{

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

    // zefiroControl->return_errors[0] = 0.0;
    // zefiroControl->return_errors[1] = 0.0;
    // zefiroControl->return_errors[2] = 0.0;

    gps().init(zefiroControl->serial_manager);
    gps().update();

    //actuador->set_motor_limits(1000, 2000);
    zefiroControl->set_manual(2);

    return true;
}

void ModeManualHover::_exit()
{

}

void ModeManualHover::update()
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
        control_throttle = (control_throttle - zero_throttle) / normalizador_throttle;
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
    float Z = -state.Z;

    zefiroControl->update(U, V, W,
                        P, Q, R,
                        roll, pitch, yaw,
                        X, Y, Z);

    double force_vector[12];
    zefiroControl->get_force_vector(force_vector);

    actuador->update(force_vector);
    
}


