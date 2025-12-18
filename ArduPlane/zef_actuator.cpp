#include <AP_HAL/AP_HAL.h>
#include "zef_actuator.h"
#include "zef_control.h"

using namespace AP;

zefActuador::zefActuador(ActuatorType type) :
    _pwm_servo_min(500),
    _pwm_servo_max(2500),
    _pwm_min_motor(1000),
    _pwm_max_motor(1700),
    _force_min(1.0),  
    _force_max(30.0), 
    _max_angle(225.0), 
    _pi_value(3.14159265358979311600),
    _gpio_pin(50),
    _inflator_state(0),
    _inflator1_state(0),
    _inflator2_state(0)
{
    // Initialize outputs to safe values
    set_actuator_coefficients(type);
    _current_outputs = {};
    _current_outputs.motors_armed = false;
}

void zefActuador::set_actuator_coefficients(ActuatorType type) {
    switch(type) {
        case ACTUATOR_DFDL:
            _coefficients.coeficiente_quadratico_forca = -0.1084294;
            _coefficients.coeficiente_linear_forca = 12.8454;
            _coefficients.coeficiente_nulo_forca = 1195;
            break;
            
        case ACTUATOR_APC1412xAPC1414:
            _coefficients.coeficiente_quadratico_forca = -0.1084294; 
            _coefficients.coeficiente_linear_forca = 12.8454;        
            _coefficients.coeficiente_nulo_forca = 1195;             
            break;
            
        default:
            // Default to DFDL
            _coefficients.coeficiente_quadratico_forca = -0.1084294;
            _coefficients.coeficiente_linear_forca = 12.8454;
            _coefficients.coeficiente_nulo_forca = 1195;
            break;
    }
}

void zefActuador::update(const double (&force_vector)[12]) {
    
    // A force vector U is expected to be provided by the control system
    // Then there must be a method to set the angles and forces based on this vector

    set_force(force_vector);
    set_angles(force_vector);

    calculate_pwm_force_motors();
    calculate_pwm_tilt_servos();

    update_current_outputs();

    send_motor_commands();
    send_servo_commands();

}

void zefActuador::calculate_pwm_tilt_servos() {
    
    _pwm_tilt_01 = angle2PWM_servo(_tilt_01);
    
    _pwm_tilt_02 = angle2PWM_servo(_tilt_02);
    
    _pwm_tilt_03 = angle2PWM_servo(_tilt_03);
    
    _pwm_tilt_04 = angle2PWM_servo(_tilt_04);
}

void zefActuador::calculate_pwm_force_motors() {
    
    _current_outputs.motors_armed = AP::arming().is_armed();
    
    if (_current_outputs.motors_armed) {

        _pwm_force_01 = force2PWM_motor(_force_01);
        
        _pwm_force_02 = force2PWM_motor(_force_02);
        
        _pwm_force_03 = force2PWM_motor(_force_03);
        
        _pwm_force_04 = force2PWM_motor(_force_04);
        
    } else {
        _pwm_force_01 = 0;
        _pwm_force_02 = 0;
        _pwm_force_03 = 0;
        _pwm_force_04 = 0;
    }
}

double zefActuador::set_angle_range(double last_angle, double v1, double v0) {

    double pi = _pi_value; 
    double max_angle_rad = _max_angle * pi / 180.0;

    double new_angle = atan2f(v1, v0);

    if (std::abs(new_angle - last_angle) > pi) {
        if (new_angle < last_angle) {
            new_angle += 2 * pi;
        } else {
            new_angle -= 2 * pi;
        }
    }

    if (new_angle > max_angle_rad) {
        new_angle -= 2 * pi;
    } else if (new_angle < -max_angle_rad) {
        new_angle += 2 * pi;
    }

    return new_angle;
}

void zefActuador::set_force(const double (&U)[12]) {

    _force_01 = sqrtf(powf(U[0], 2) + powf(U[1], 2));

    _force_02 = sqrtf(powf(U[2], 2) + powf(U[3], 2));

    _force_03 = sqrtf(powf(U[4], 2) + powf(U[5], 2));

    _force_04 = sqrtf(powf(U[6], 2) + powf(U[7], 2));

}

void zefActuador::set_angles(const double (&U)[12]){

    if( _force_01 >= _force_min ) {
        _tilt_01 = set_angle_range(_tilt_01, U[1], U[0]);
    }

    if( _force_02 >= _force_min ) {
        _tilt_02 = set_angle_range(_tilt_02, U[3], U[2]);
    }

    if( _force_03 >= _force_min ) {
        _tilt_03 = set_angle_range(_tilt_03, U[5], U[4]);
    }

    if( _force_04 >= _force_min ) {
        _tilt_04 = set_angle_range(_tilt_04, U[7], U[6]);
    }

}

int zefActuador::angle2PWM_servo(double servo_angle) {

    //double max_angle_rad = _max_angle * _pi_value / 180.0;
    //double angular_coefficient = (double)(_pwm_servo_max - _pwm_servo_min) / (2 * max_angle_rad);
    //int retVal = (int)((servo_angle + max_angle_rad) * angular_coefficient + _pwm_servo_min);

     int retVal = (double)208.93*servo_angle + 1500.0;  // Default Curve, where 1500us points to 0 degrees

    if (retVal > _pwm_servo_max) retVal = _pwm_servo_max;
    if (retVal < _pwm_servo_min) retVal = _pwm_servo_min;

    return retVal;
}

int zefActuador::force2PWM_motor(double motor_force) {
    double command = 0.0;

    if( motor_force >= _force_min ) {
        command = (_coefficients.coeficiente_quadratico_forca * powf(motor_force, 2)) + 
                    (_coefficients.coeficiente_linear_forca * motor_force) + 
                    _coefficients.coeficiente_nulo_forca;
    }

    int retVal = (int)command;
    if (retVal > _pwm_max_motor) retVal = _pwm_max_motor;
    if (retVal < _pwm_min_motor) retVal = _pwm_min_motor;

    return retVal;
}

float zefActuador::dead_zone(double input_value, double dead_zone_limit) {
    double retVal = input_value;
    if ((input_value < dead_zone_limit) && (input_value > -dead_zone_limit)) {
        retVal = 0.0;
    }
    return retVal;
}

void zefActuador::stopInflators() {
    _inflator_state = 0;
    hal.gpio->pinMode(_gpio_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_gpio_pin, 0);
    hal.gpio->pinMode(_gpio_pin + 1, HAL_GPIO_OUTPUT);
    hal.gpio->write(_gpio_pin + 1, 0);
}

void zefActuador::operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit) {
    double pressure_diff = (barometer->get_pressure(1) - barometer->get_pressure(0)) * 0.01;
    
    if (pressure_diff < min_p && _inflator_state == 0) {
        _inflator_state = 1;
        hal.gpio->pinMode(_gpio_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_gpio_pin, 1);
        hal.gpio->pinMode(_gpio_pin + 1, HAL_GPIO_OUTPUT);
        hal.gpio->write(_gpio_pin + 1, 1);
    } else {
        if (_inflator_state == 1 && pressure_diff > max_p) {
            _inflator_state = 0;
        }
        if (_inflator_state == 1) {
            hal.gpio->pinMode(_gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(_gpio_pin, 1);
            hal.gpio->pinMode(_gpio_pin + 1, HAL_GPIO_OUTPUT);
            hal.gpio->write(_gpio_pin + 1, 1);
        } else {
            hal.gpio->pinMode(_gpio_pin, HAL_GPIO_OUTPUT);
            hal.gpio->write(_gpio_pin, 0);
            hal.gpio->pinMode(_gpio_pin + 1, HAL_GPIO_OUTPUT);
            hal.gpio->write(_gpio_pin + 1, 0);
        }
    }
}

void zefActuador::send_servo_commands() {

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor_tilt, _current_outputs.pwm_d1_angulo);
    SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRearRight, _current_outputs.pwm_d2_angulo);
    SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRear, _current_outputs.pwm_d3_angulo);
    SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRearLeft, _current_outputs.pwm_d4_angulo);

}

void zefActuador::send_motor_commands() {

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, _current_outputs.pwm_motor_F1);
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, _current_outputs.pwm_motor_F2);
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, _current_outputs.pwm_motor_F3);
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, _current_outputs.pwm_motor_F4);

}

void zefActuador::set_servo_limits(int16_t min, int16_t max) {
    _pwm_servo_min = min;
    _pwm_servo_max = max;
}

void zefActuador::set_motor_limits(int16_t min, int16_t max) {
    _pwm_min_motor = min;
    _pwm_max_motor = max;
}

void zefActuador::update_current_outputs() {
    // Update servo/tilt PWM values
    _current_outputs.pwm_d1_angulo = _pwm_tilt_01;
    _current_outputs.pwm_d2_angulo = _pwm_tilt_02;
    _current_outputs.pwm_d3_angulo = _pwm_tilt_03;
    _current_outputs.pwm_d4_angulo = _pwm_tilt_04;
    
    // Update force values
    _current_outputs.pwm_motor_F1 = _pwm_force_01;  
    _current_outputs.pwm_motor_F2 = _pwm_force_02;
    _current_outputs.pwm_motor_F3 = _pwm_force_03;
    _current_outputs.pwm_motor_F4 = _pwm_force_04;
    
    // Update armed status
    _current_outputs.motors_armed = AP::arming().is_armed();
}

void zefActuador::set_motor_servo2origin(){

    _current_outputs.pwm_d1_angulo = 1500;
    _current_outputs.pwm_d2_angulo = 1500;
    _current_outputs.pwm_d3_angulo = 1500;
    _current_outputs.pwm_d4_angulo = 1500;

    _current_outputs.pwm_motor_F1 = 1000;  
    _current_outputs.pwm_motor_F2 = 1000;
    _current_outputs.pwm_motor_F3 = 1000;
    _current_outputs.pwm_motor_F4 = 1000;

    send_servo_commands();
    send_motor_commands();
}

zefActuador::ActuatorOutputs zefActuador::get_outputs() const {
    return _current_outputs;
}

zefActuador* zefActuador::instancePtr = nullptr;
zefActuador zefiroActuador;