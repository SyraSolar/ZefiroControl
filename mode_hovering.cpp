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
//#include "quadplane.h"
#include "zef_control.h"

using namespace AP;

// Serial manager is needed for UART communications
/*static AP_SerialManager serial_manager;*/



//static uint16_t pwm = 1500;
//static int8_t delta = 1;

bool ModeHovering::_enter()
{
    //barometer.init();
    //barometer.calibrate();

    /*for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }*/

    // Initialize the UART for GPS system
    //serial_manager.init();
    zefiroControl.return_errors[0] = 0.0;
    zefiroControl.return_errors[1] = 0.0;
    zefiroControl.return_errors[2] = 0.0;

    gps().init(zefiroControl.serial_manager);
    gps().update();
    zefiroControl.must_reset_loc = true; //semaforo para resetar posição

    zefiroControl.set_manual(0);

    return true;
}

void ModeHovering::_exit()
{
    /*for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->disable_ch(i);
    }*/
    //zefiroControl.stopInflators();
}

void ModeHovering::update()
{
    static uint32_t last_msg_ms;
    // Update GPS state based on possible bytes received from the module.
    gps().update();

    ahrs().update();
    
    // Obter matriz de atitude (matriz de rotação corpo-terra)
    //Matrix3f attitude_matrix = ahrs().get_rotation_body_to_ned().transposed();
    
    
    //quadplane.update();
    /*Vector3f curr_pos_rel_ekf = zefiroControl.zef_inertial_nav.get_position_neu_cm();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, " x: %.2fm \n"
                                     " y: %.2fm \n"
                                     " z: %.2fm \n",
                                    (double)curr_pos_rel_ekf[0],
                                    (double)curr_pos_rel_ekf[1],
                                    (double)curr_pos_rel_ekf[2]);*/
    
    
    float pitch = ahrs().get_pitch(); // Get pitch angle in degrees
    float yaw = ahrs().get_yaw(); // Get yaw angle in degrees
    float roll = ahrs().get_roll(); // Get roll angle in degrees

    // If new GPS data is received, output its contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    Location loc;
    double alt_barometer = ((1013.25 - (plane.barometer.get_pressure()*0.01))*30)*0.3048;
    if (last_msg_ms != gps().last_message_time_ms()) {
        // Reset the time of message
        last_msg_ms = gps().last_message_time_ms();
        if( zefiroControl.must_reset_loc ) {
            zefiroControl.must_reset_loc = false;
            //desired_position = gps().location();
            ahrs().EKF3.getLLH(zefiroControl.desired_position);
            zefiroControl.desired_alt = alt_barometer;
        }

        // Acquire location
        ahrs().get_location(loc);
        //float ground_speed = gps().ground_speed(); // Get ground speed in meters per second*/

        // Print the contents of message
        /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, " Lat: %.2fm \n"
                                        " Lon: %.2fm \n"
                                        " Alt: %.2fm \n"
                                        " Ground speed: %.2fm \n",
                                        (double)loc.lat,
                                        (double)loc.lng,
                                        (double)(loc.alt * 0.01f),
                                        (double)ground_speed);*/
                                        

        zefiroControl.getPositionError((double) zefiroControl.desired_position.lat, (double) zefiroControl.desired_position.lng, (double)(zefiroControl.desired_alt), (double) loc.lat, (double) loc.lng, (double)(alt_barometer), (double) yaw, zefiroControl.return_errors);
        //zefiroControl.getPositionError((double) desired_position.lat, (double) desired_position.lng, (double)(desired_position.alt), (double) loc.lat, (double) loc.lng, (double)(loc.alt), (double) yaw, zefiroControl.return_errors);
    }
    
    Vector3f inertial_vector((float) zefiroControl.return_errors[0], (float) zefiroControl.return_errors[1], (float) zefiroControl.return_errors[2]);
    Vector3f body_vector = zefiroControl.rotate_inertial_to_body(roll, pitch, yaw, inertial_vector);
    
    zefiroControl.set_RHO(plane.barometer.get_pressure()*0.00001);

    /*if (barometer.healthy()) {
        //output barometer readings to console
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, " Pressure: %.2f Pa\n"
                            " Temperature: %.2f degC\n"
                            " Relative Altitude: %.2f m\n"
                            " climb=%.2f m/s\n"
                            "\n",
                            (double)barometer.get_pressure(),
                            (double)barometer.get_temperature(),
                            (double)barometer.get_altitude(),
                            (double)barometer.get_climb_rate());
    }*/

    //plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch: %f --- yaw: %f --- roll: %f", pitch, yaw, roll);

    // Read angular rates for roll, pitch, and yaw
    Vector3f angular_gyro = ahrs().get_gyro();
    float angular_pitch = angular_gyro.y; // Get angular rate for pitch in degrees per second
    float angular_roll = angular_gyro.x; // Get angular rate for roll in degrees per second
    float angular_yaw = angular_gyro.z; // Get angular rate for yaw in degrees per second
    
    Vector3f vel_NED;
    bool ret_NED = ahrs().get_velocity_NED(vel_NED);

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang_pitch: %f --- ang_yaw: %f --- ang_roll: %f", angular_pitch, angular_yaw, angular_roll);

    //zefiroControl.update(double U_longit_speed, double V_lateral_speed, double W_vertical_speed,
    //    double P_v_roll, double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw);
    double wind_direction = wrap_PI(plane.g2.windvane.get_apparent_wind_direction_rad()); // radians(wrap_360(degrees(plane.g2.windvane.get_apparent_wind_direction_rad())));
    double wind_speed = plane.g2.windvane.get_apparent_wind_speed();
    double vx = wind_speed * cosf(wind_direction);
    double vy = wind_speed * sinf(wind_direction);
    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "2 >>> wind dir: %f --- wind speed: %f", degrees(wind_direction), wind_speed);

    double U_longit_speed = vx;
    double V_lateral_speed = vy;
    double W_vertical_speed = plane.barometer.get_climb_rate(); 
    if(ret_NED) W_vertical_speed = vel_NED.z; //plane.barometer.get_climb_rate();
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gps alt: %f --- baro climb: %f --- vert_vel: %f", (double)loc.alt, (double)plane.barometer.get_climb_rate(), (double)W_vertical_speed);
    double P_v_roll = angular_roll;
    double Q_v_pitch = angular_pitch;
    double R_v_yaw = angular_yaw;

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gps alt: %f --- baro alt: %f", (double)loc.alt, (double)barometer.get_altitude());

    zefiroControl.update(U_longit_speed, V_lateral_speed, W_vertical_speed,
        P_v_roll, Q_v_pitch, R_v_yaw,
        roll, pitch, yaw,
        body_vector.x, body_vector.y, alt_barometer); //body_vector.z); //loc.lat, loc.lng, loc.alt);

    int16_t angle_min = 500;
    int16_t angle_max = 2500;

    //double d1_angulo_motor, d2_angulo_motor, d3_angulo_motor, d4_angulo_motor;
    //double dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo, dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo;
    int16_t pwm_d1_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d1_angulo_motor, angle_min, angle_max);
    int16_t pwm_d2_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d2_angulo_motor, angle_min, angle_max);
    int16_t pwm_d3_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d3_angulo_motor, angle_min, angle_max);
    int16_t pwm_d4_angulo = zefiroControl.get_value_to_pwm_servo(zefiroControl.d4_angulo_motor, angle_min, angle_max);

    // SRV_Channels::move_servo(SRV_Channel::k_motor_tilt, output_d1_angulo, angle_min, angle_max);
    // SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearRight, output_d2_angulo, angle_min, angle_max);
    // SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRear, output_d3_angulo, angle_min, angle_max);
    // SRV_Channels::move_servo(SRV_Channel::k_tiltMotorRearLeft, output_d4_angulo, angle_min, angle_max);

    SRV_Channels::tilt_servo(SRV_Channel::k_motor_tilt, pwm_d1_angulo);
    SRV_Channels::tilt_servo(SRV_Channel::k_tiltMotorRearRight, pwm_d2_angulo);
    SRV_Channels::tilt_servo(SRV_Channel::k_tiltMotorRear, pwm_d3_angulo);
    SRV_Channels::tilt_servo(SRV_Channel::k_tiltMotorRearLeft, pwm_d4_angulo);

    // printf(
    //     "S>>%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f," // Dynamical State
    //     "%.4f,%.4f,%.4f,%.4f,"                                        // Servo Reference Angles (assuming these are doubles/floats)
    //     "%d,%d,%d,%d,"                                                // Servo PWM values (int16_t, so use %d)
    //     "%.4f,%.4f,%.4f,%.4f,"                                        // Motor Reference Forces (assuming these are doubles/floats)
    //     "%d,%d,%d,%d\n",                                              // Motor Command PWMs (int16_t, so use %d)
    //     U_longit_speed, V_lateral_speed, W_vertical_speed,
    //     P_v_roll, Q_v_pitch, R_v_yaw,
    //     roll, pitch, yaw,
    //     body_vector.x, body_vector.y, alt_barometer,
    //     zefiroControl.d1_angulo_motor, zefiroControl.d2_angulo_motor,
    //     zefiroControl.d3_angulo_motor, zefiroControl.d4_angulo_motor,
    //     pwm_d1_angulo, pwm_d2_angulo, pwm_d3_angulo, pwm_d4_angulo,
    //     zefiroControl.F1_forca_motor, zefiroControl.F2_forca_motor,
    //     zefiroControl.F3_forca_motor, zefiroControl.F4_forca_motor,
    //     zefiroControl.comando_M1, zefiroControl.comando_M2,
    //     zefiroControl.comando_M3, zefiroControl.comando_M4
    // );


    //zefiroControl.operateInflators(&plane.barometer, plane.g2.min_balloon_pressure_diff, plane.g2.max_balloon_pressure_diff);

    //controles da empenagem
    /*int16_t output_vert_stabilizer = zefiroControl.get_value_to_pwm_servo(zefiroControl.dvu_ang_estab_vert_cima, angle_min, angle_max);
    int16_t output_hor_stabilizer = zefiroControl.get_value_to_pwm_servo(zefiroControl.dhr_ang_estab_horiz_direito, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_rudder, output_vert_stabilizer, angle_min, angle_max);
    SRV_Channels::move_servo(SRV_Channel::k_elevator, output_hor_stabilizer, angle_min, angle_max);*/


    int16_t min_motor = 1000;
    int16_t max_motor = 2000; //2000;
    if( AP::arming().is_armed() ) {
        /*int F1 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F1_forca_motor, min_motor, max_motor);
        int F2 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F2_forca_motor, min_motor, max_motor);
        int F3 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F3_forca_motor, min_motor, max_motor);
        int F4 = zefiroControl.get_value_to_pwm_motor(zefiroControl.F4_forca_motor, min_motor, max_motor);*/

        int F1 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M1, min_motor, max_motor);
        int F2 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M2, min_motor, max_motor);
        int F3 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M3, min_motor, max_motor);
        int F4 = zefiroControl.get_value_to_pwm_motor(zefiroControl.comando_M4, min_motor, max_motor);

        SRV_Channels::move_servo(SRV_Channel::k_motor1, F1, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor2, F2, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor3, F3, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor4, F4, min_motor, max_motor);
    } else {
        SRV_Channels::move_servo(SRV_Channel::k_motor1, 0, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor2, 0, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor3, 0, min_motor, max_motor);
        SRV_Channels::move_servo(SRV_Channel::k_motor4, 0, min_motor, max_motor);
    }


    //plane.nav_roll_cd = plane.ahrs.roll_sensor;
    //plane.nav_pitch_cd = plane.ahrs.pitch_sensor;

}


