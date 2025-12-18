
#include "mode.h"
#include "Plane.h"

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

#include "zef_observer.h"
#include "zef_control.h"
#include "zef_actuator.h"

using namespace AP;

bool ModeFBWB::_enter()
{
    if (!observer) {
        observer = zefObserver::getInstance(plane.ahrs);
    }
    if (!zefiroControl) {
        zefiroControl = ZefControl::getInstance();
        zefiroControl->set_matrix_index(25);
    }
    if (!actuador) {
        actuador = zefActuador::getInstance();
    }

    observer->_baro_reference_set = false;
    last_print_ms = 0;

    gps().init(zefiroControl->serial_manager);
    gps().update();

    observer->update();
    zefObserver::AircraftState state = observer->get_state();

    zefiroControl->set_position_reference(state.X, state.Y, state.Z);
    zefiroControl->set_manual(0);

    zefiroControl->set_accumulative_error();
    zefiroControl->set_roll_reference(0.0);

    mode_entry_time_ms = AP_HAL::millis();

    Att_counter = 0.0;

    return true;
}

void ModeFBWB::update()
{

    uint32_t now = AP_HAL::millis();
    uint32_t time_in_mode = now - mode_entry_time_ms;
    
    _last_update_ms = now;

    if (time_in_mode < 5000) {
        zefiroControl->set_accumulative_error();
    }

    // update observer and control
    observer->update();
    zefObserver::AircraftState state = observer->get_state();

    double U = state.vx;
    double V = state.vy;
    double W = state.vz;
    double P = state.angular_roll;
    double Q = state.angular_pitch;
    double R = state.angular_yaw;
    double roll  = state.roll;
    double pitch = state.pitch;
    double yaw   = state.yaw;
    double X = state.X;
    double Y = state.Y;
    double Z = state.Z;

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

    //uint32_t now = AP_HAL::millis();
    if (now - last_print_ms > 20) {
        last_print_ms = now;
        hal.console->printf(
            "S>>%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
            "%d,%d,%d,%d,"
            "%d,%d,%d,%d,"
            "%.1f,%.1f,%.1f,%.1f,"
            "%.1f,%.1f,%.1f,%.1f\n",
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