#include <math.h>

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>
//#include "Plane.h"
#include "zef_k_matrix.h"

class ZefControl {
private:
    //initialization
    ZefControl() {
        memcpy(k_matrix_all, zef_matrix, sizeof(zef_matrix));
        _last_update_ms = 0;
    }
    static ZefControl* instancePtr;

    uint32_t _last_update_ms;

    double antagonist_force = 3.0; //*3.5E+00;
    
    double F_max_mot = 21.0; // [N] -> Equivale a 2.1gf and 1800us
    double F_min_mot = 2.9; // [N] -> Equivale a 300gf
    
    double comando_minimo = 1000.0; 
    double comando_maximo = 2000.0; //1800.0;
    
    double coeficiente_quadratico_forca = -0.1084294;
    double coeficiente_linear_forca = 12.8454;
    double coeficiente_nulo_forca = 1195.0;

    int current_matrix_index = 25;  // Add this member variable

    double pi_value = 3.14159265358979311600;
    
    int ref_index = 0; // curr index for the bellow array
    double speed_refs[40] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0,31.0,32.0,33.0,34.0,35.0,36.0,37.0,38.0,39.0}; //speeds references to change the active K matrix
    
    double RHO_air_density = 1.2; //densidade do ar
    
    int gpio_pin = 50;
    int inflator_state = 0;
    int inflator1_state = 0; //independent state for pressure balloons
    int inflator2_state = 0;
    
    float k_matrix_all[40][12][18] = {};
    
    void find_matrix(const double longit_speed);
    void add_traction(double longit_speed);
    void antagonist_force_update();

    int is_manual_mode = 0;
    void adjust_for_manual(double (&U_array)[12]);
    void print_output_data();
    double last_X[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,};
    double Q[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
    double E_prev[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};

    double X_ref[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // double U[12];

    int semaforo = 0;

public:
    friend class Parameters;
    friend class ParametersG2;
    friend class Plane;

    // Static method to get the Singleton instance
    static ZefControl* getInstance() {
        if (instancePtr == nullptr) {
            instancePtr = new ZefControl();
        }
        return instancePtr;
    }
    
    double U[12];
    
    void get_force_vector(double (&output)[12]) const {
        memcpy(output, U, sizeof(U));
    }

    double J1_cmd_roll, J2_cmd_pitch, J3_cmd_throttle, J4_cmd_yaw, RS_cmd_up_down, LS_cmd_letf_right = 0.0;
    double F1_forca_motor, F2_forca_motor, F3_forca_motor, F4_forca_motor = 0.0;
    double comando_M1, comando_M2, comando_M3, comando_M4 = 0.0;
    double d1_angulo_motor, d2_angulo_motor, d3_angulo_motor, d4_angulo_motor = 0.0;
    double dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo, dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo = 0.0;

    void mult_mat(const float matrix[8][18], const double vector[12], double (&results)[12]);
    
    void set_manual(int state);
    
    void set_RHO(double rho);
    
    void manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch);
    
    void update( double U_longit_speed,
        double V_lateral_speed, double W_vertical_speed, double P_v_roll, 
        double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw,
        double x_position_n_s, double y_position_e_w, double z_position_height);
        
    Vector3f rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector);
    void getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]);
    
    void set_position_reference(double x_ref, double y_ref, double z_ref);
    void set_roll_reference(double roll_ref);

    double set_angle_range(double last_angle, double v1, double v0);
    double get_engine_command(double power);
    //void set_power_and_angles(double (&U)[12]);
    void set_power_and_angles();
    void put_forces_in_range();

    int get_value_to_pwm_servo(double in_value, int min_pwm, int max_pwm);
    int get_value_to_pwm_motor(double in_value, int min_pwm, int max_pwm);

    float dead_zone(double input_value, double dead_zone_limit);
    
    void set_matrix_index(int index);  
    void set_accumulative_error();
    int get_matrix_index() const;

    //specific variables for ardupilot
    AP_SerialManager serial_manager;
    double return_errors[3];
    AP_InertialNav zef_inertial_nav = AP_InertialNav(AP::ahrs());
    bool must_reset_loc;
    Location desired_position;
    double desired_alt;

    void Attitude_Proportional_Control(double vector_error[18]);
    double U_prop_att[12];

    void Attitude_Integral_Control(double vector_error[18]);
    double U_int_att[12];

    void Altitude_Proportional_Control(double vector_error[18]);
    double U_prop_alt[12];

    void Altitude_Integral_Control(double vector_error[18]);
    double U_int_alt[12];

    void Position_Proportional_Control(double vector_error[18]);
    double U_prop_pos[12];

    void Position_Integral_Control(double vector_error[18]);
    double U_int_pos[12];

    double U_eq[12];

};

extern ZefControl zefiroControl;
