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

bool ModeHovering::_enter()
{

    if (!observer) {
        observer = zefObserver::getInstance(plane.ahrs); //new zefObserver(plane.ahrs);
    }
    if (!zefiroControl) {
        zefiroControl = ZefControl::getInstance();
        zefiroControl->set_matrix_index(25);
        zefiroControl->set_roll_reference(0.0);
    }
    if (!actuador) {
        actuador = zefActuador::getInstance(); //actuador = new zefActuador();
    }

    observer->_baro_reference_set = false;

    last_print_ms = 0;

    // zefiroControl->return_errors[0] = 0.0;
    // zefiroControl->return_errors[1] = 0.0;
    // zefiroControl->return_errors[2] = 0.0;

    gps().init(zefiroControl->serial_manager);
    gps().update();

    observer->update();
    
    zefObserver::AircraftState state = observer->get_state();

    zefiroControl->set_position_reference(state.X, state.Y, state.Z);

    //actuador->set_motor_limits(1000, 1700);
    zefiroControl->set_manual(0);

    return true;
}

void ModeHovering::_exit()
{

}

void ModeHovering::update()
{
    
    if (zefiroControl) {
        zefiroControl->set_matrix_index(25); 
        zefiroControl->set_roll_reference(0.0);
        zefiroControl->set_accumulative_error();
    }

    observer->update();
    
    zefObserver::AircraftState state = observer->get_state();

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

    double eU = X_ref[0] - U;
    double eV = X_ref[1] - V;
    double eW = X_ref[2] - W;   
    double eP = X_ref[3] - P;
    double eQ = X_ref[4] - Q;
    double eR = X_ref[5] - R;
    double eroll  = X_ref[6] - roll;
    double epitch = X_ref[7] - pitch;
    double eyaw   = X_ref[8] - yaw;
    double eX = X_ref[9]  - X;
    double eY = X_ref[10] - Y;
    double eZ = X_ref[11] - Z;

    zefiroControl->update(eU, eV, eW,
                        eP, eQ, eR,
                        eroll, epitch, eyaw,
                        eX, eY, eZ);

    double force_vector[12];
    zefiroControl->get_force_vector(force_vector);

    actuador->update(force_vector);
    zefActuador::ActuatorOutputs outputs = actuador->get_outputs();

    uint32_t now = AP_HAL::millis();
    
    if (now - last_print_ms > 20) {  
        last_print_ms = now;
        
        // Send telemetry data via Serial
        hal.console->printf(
            "S>>%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f," 
            "%d,%d,%d,%d,"                                                    
            "%d,%d,%d,%d,"                                                    
            "%.1f,%.1f,%.1f,%.1f,"                                            
            "%.1f,%.1f,%.1f,%.1f\n",                                        
            //
            U, V, W,
            P, Q, R,
            roll, pitch, yaw,
            X, Y, Z,
            outputs.pwm_d1_angulo, outputs.pwm_d2_angulo, outputs.pwm_d3_angulo, outputs.pwm_d4_angulo,
            outputs.pwm_motor_F1, outputs.pwm_motor_F2, outputs.pwm_motor_F3, outputs.pwm_motor_F4,
            force_vector[0], force_vector[1], force_vector[2], force_vector[3],
            force_vector[4], force_vector[5], force_vector[6], force_vector[7]
        );
    }


}


