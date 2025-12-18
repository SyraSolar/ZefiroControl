#ifndef ZEFIRO_ACTUADOR_H
#define ZEFIRO_ACTUADOR_H

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Baro/AP_Baro.h>

class ZefControl;

class zefActuador {
public:
    
    enum ActuatorType {
        ACTUATOR_DFDL,
        ACTUATOR_APC1412xAPC1414,
        ACTUATOR_DEFAULT = ACTUATOR_DFDL
    };
    zefActuador(ActuatorType type = ACTUATOR_DEFAULT);

    // Static method to get the Singleton instance
    static zefActuador* getInstance(ActuatorType type = ACTUATOR_DEFAULT) {
        if (instancePtr == nullptr) {
            instancePtr = new zefActuador(type);
        }
        return instancePtr;
    }

    void update(const double (&force_vector)[12]);
    void calculate_pwm_tilt_servos();
    void calculate_pwm_force_motors();
    
    void set_servo_limits(int16_t min, int16_t max);
    void set_motor_limits(int16_t min, int16_t max);

    int angle2PWM_servo(double servo_angle);
    int force2PWM_motor(double motor_force);
    float dead_zone(double input_value, double dead_zone_limit);

    double set_angle_range(double last_angle, double v1, double v0);
    void set_force(const double (&U)[12]);
    void set_angles(const double (&U)[12]);

    void operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit);
    void stopInflators();
    
    void set_motor_servo2origin();
    
    struct ActuatorOutputs {
        int16_t pwm_d1_angulo, pwm_d2_angulo, pwm_d3_angulo, pwm_d4_angulo;
        int16_t pwm_motor_F1, pwm_motor_F2, pwm_motor_F3, pwm_motor_F4;
        bool motors_armed;
    };
    
    ActuatorOutputs get_outputs() const;

private:

    static zefActuador* instancePtr;

    struct ActuatorCoefficients {
        double coeficiente_quadratico_forca;
        double coeficiente_linear_forca;
        double coeficiente_nulo_forca;
    };

    ActuatorOutputs _current_outputs;
    void update_current_outputs();  
    
    ActuatorType _actuator_type;
    ActuatorCoefficients _coefficients;
    
    void set_actuator_coefficients(ActuatorType type);

    // Servo limits
    int16_t _pwm_servo_min;
    int16_t _pwm_servo_max;
    
    // Motor limits
    int16_t _pwm_min_motor;
    int16_t _pwm_max_motor;

    // Force limits
    double _force_min;
    double _force_max;

    // Servo angles
    double _max_angle;

    double _pi_value;
    
    // GPIO pin for inflators
    int _gpio_pin;
    int _inflator_state;
    int _inflator1_state;
    int _inflator2_state;

    float _force_01, _force_02, _force_03, _force_04;
    float _pwm_force_01, _pwm_force_02, _pwm_force_03, _pwm_force_04;
    float _tilt_01, _tilt_02, _tilt_03, _tilt_04;
    float _pwm_tilt_01, _pwm_tilt_02, _pwm_tilt_03, _pwm_tilt_04;
    
    // Helper functions
    void send_servo_commands();
    void send_motor_commands();
};

#endif // ZEFIRO_ACTUADOR_H

extern zefActuador zefiroActuador;