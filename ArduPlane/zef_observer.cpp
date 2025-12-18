#include "zef_observer.h"
#include <AP_Common/AP_Common.h>
#include <cmath>

#include "mode.h"
#include "Plane.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_NavEKF3/Ap_NavEKF3.h>
#include <AP_Baro/AP_Baro.h>

extern Plane plane;
#define gps AP::gps

using namespace AP;

zefObserver::zefObserver(AP_AHRS &ahrs)
    : _ahrs(ahrs), _last_msg_ms(0),
      _baro_reference_altitude(0.0f), _baro_reference_set(false) {}
       
zefObserver::AircraftState zefObserver::get_state() const {
    AircraftState state;

    // Core attitude and rates
    state.roll = _roll;
    state.pitch = _pitch;
    state.yaw = _yaw;

    // Angular rates
    state.angular_roll = _angular_roll;
    state.angular_pitch = _angular_pitch;
    state.angular_yaw = _angular_yaw;
    
    // Velocity and position
    state.vx = _vx;
    state.vy = _vy;
    state.vz = _vz;

    // Position in NED frame, but with Z positive upwards
    state.X = _X;
    state.Y = _Y;
    state.Z = _Z;
    
    // Additional data
    state.altitude = _altitude;
    state.climb_rate = _climb_rate;
    state.wind_speed = _wind_speed;
    state.wind_direction = _wind_direction;
    state.current_location = _current_location;
    
    return state;
}

void zefObserver::update() {
    gps().update();
    _ahrs.update();

    get_attitude();
    
    get_ned_position();
    get_ned_velocity();
}

void zefObserver::get_orientation() {
    
    // Check if GPS is available and AHRS is healthy
    // bool gps_available = (gps().status() >= AP_GPS::GPS_OK_FIX_3D);
    // bool ahrs_healthy = _ahrs.healthy();
    
    // Send the AHRS and GPS health status to the console
    // hal.console->printf("AHRS healthy: %s\n", ahrs_healthy ? "true" : "false");
    // hal.console->printf("GPS available: %s\n", gps_available ? "true" : "false");

    // Use full AHRS with GPS fusion (best accuracy)
    _roll = _ahrs.get_roll();
    _pitch = _ahrs.get_pitch();
    _yaw = _ahrs.get_yaw();

}

void zefObserver::get_angular_rates() {

    Vector3f angular_gyro = AP::ahrs().get_gyro();

    _angular_roll = angular_gyro.x;
    _angular_pitch = angular_gyro.y;
    _angular_yaw = angular_gyro.z;

    // I think this is less efficient 
    // than using the above
    // _angular_roll = _ahrs.get_gyro().x;
    // _angular_pitch = _ahrs.get_gyro().y;
    // _angular_yaw = _ahrs.get_gyro().z;

}

void zefObserver::get_attitude() {

    get_orientation();
    get_angular_rates();

}

void zefObserver::get_attitude_and_altitude() {

    get_attitude();
    get_altitude();

}

void zefObserver::get_BARO_altitude() {

    float pressure_mbar = plane.barometer.get_pressure() * 0.01f;
    _baro_altitude = calculate_altitude_from_pressure(pressure_mbar);
    _baro_climb_rate = plane.barometer.get_climb_rate();

    // Initialize reference altitude on first reading
    if (!_baro_reference_set) {
        _baro_reference_altitude = _baro_altitude;
        _baro_reference_set = true;
        _Z = 0.0f; // Start at zero relative position
    } else {
        // Calculate relative position from reference (positive upward)
        _Z = - _baro_altitude + _baro_reference_altitude; // Positive when above reference
    }
    
    _vz = - _baro_climb_rate; // Convert climb rate to NED convention (positive down)
}

void zefObserver::get_GPS_altitude() {

    // Check if GPS has a valid 3D fix
    if (gps().status() >= AP_GPS::GPS_OK_FIX_3D) {

        _gps_altitude = gps().location().alt * 0.01f; // cm to meters
        _gps_climb_rate = gps().velocity().z; // m/s

        _Z = _gps_altitude; 
        _vz = _gps_climb_rate; 

    } else {
        // Let it be...
    }
}

void zefObserver::get_altitude() {

    // Get EKF fused altitude estimate (best of GPS + Baro)
    Location current_loc;
    if (_ahrs.get_location(current_loc)) {

        _altitude = current_loc.alt * 0.01f; // Convert cm to meters

        Vector3f vel_NED;
        bool ret_NED = _ahrs.get_velocity_NED(vel_NED);
        if (ret_NED) {
            _vz = vel_NED.z;
        } else {
            // Fallback to barometer climb rate if NED velocity is not available
            _vz = - plane.barometer.get_climb_rate();
        }

    } else {
        // Fallback to barometer only if EKF fails
        float pressure_mbar = plane.barometer.get_pressure() * 0.01f;
        _altitude = calculate_altitude_from_pressure(pressure_mbar);
        _vz = - plane.barometer.get_climb_rate();
    }
}

void zefObserver::get_ned_velocity() {

    Vector3f vel_NED;
    bool ret_NED = _ahrs.get_velocity_NED(vel_NED);

    if (ret_NED) {
        _vx = vel_NED.x;
        _vy = vel_NED.y;
        _vz = vel_NED.z;
    } else {
        // Fallback to barometer climb rate if NED velocity is not available
        _vx = 0.0f; 
        _vy = 0.0f; 
        _vz = - plane.barometer.get_climb_rate();
    }

    Vector3f body_velocity = _ahrs.earth_to_body(vel_NED);

    // Quaternion q;
    // _ahrs.get_quat_body_to_ned(q);      // q is Body → NED
    // q = q.inverse();                    // Now q is NED → Body
    // Vector3f body_velocity = vel_NED;  // Make a copy
    // q.rotate(body_velocity);           // Rotates in-place

    _vx = body_velocity.x;
    _vy = body_velocity.y;
    _vz = body_velocity.z;

}

void zefObserver::get_ned_position() {
    
    Vector3f ned_position;

    // Get EKF relative position from origin in NED frame
    if (_ahrs.get_relative_position_NED_origin(ned_position)) {
        _X = ned_position.x; // North (meters)
        _Y = ned_position.y; // East (meters)  
        _Z = ned_position.z; // Convert NED Down to positive upward
    } else {
        // Fallback: try to get relative position from home
        if (_ahrs.get_relative_position_NED_home(ned_position)) {
            _X = ned_position.x;
            _Y = ned_position.y;
            _Z = ned_position.z; // Convert NED Down to positive upward
        } else {
            // Last fallback: calculate from current location vs origin
            Location origin;
            if (_ahrs.get_origin(origin) && _current_location.lat != 0) {
                // Calculate relative position manually
                Vector2f diff = origin.get_distance_NE(_current_location);
                _X = diff.x; // North
                _Y = diff.y; // East
                _Z = (origin.alt - _current_location.alt) * 0.01f; // Convert to positive upward
            } else {
                get_BARO_altitude();
            }
        }
    }

    //This is not working correctly
    Vector3f body_position = _ahrs.earth_to_body(ned_position);

    // First try using the rotation matrix
    // Matrix3f rotation_matrix = _ahrs.get_rotation_body_to_ned();
    // Vector3f body_position = rotation_matrix.mul_transpose(ned_position);

    // Second try using quaternion
    // Quaternion q;
    // _ahrs.get_quat_body_to_ned(q);      // q is Body → NED
    // q = q.inverse();                    // Now q is NED → Body
    // Vector3f body_position = ned_position;  // Make a copy
    // q.rotate(ned_position);  // Active rotation

    _X = body_position.x;
    _Y = body_position.y;
    _Z = body_position.z;
}

void zefObserver::compute_wind() {

    Vector3f vel_NED;
    bool ret_NED = _ahrs.get_velocity_NED(vel_NED);
    _vz = ret_NED ? vel_NED.z : plane.barometer.get_climb_rate();

    double wind_dir = wrap_PI(plane.g2.windvane.get_apparent_wind_direction_rad());
    double wind_speed = plane.g2.windvane.get_apparent_wind_speed();

    _wind_direction = wind_dir;
    _wind_speed = wind_speed;

    _vx = wind_speed * cosf(wind_dir);
    _vy = wind_speed * sinf(wind_dir);
}

double zefObserver::calculate_altitude_from_pressure(float pressure_mbar) const {
    // Approximate barometric altitude (ISA model)
    return (1013.25 - pressure_mbar) * 30.0 * 0.3048;
}

zefObserver* zefObserver::instancePtr = nullptr;