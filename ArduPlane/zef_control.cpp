#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>

#define HAL_USE_I2C     TRUE
#define HAL_USE_I2S     FALSE

#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/Display.h>
#include <AP_Notify/Display_tca.h>

#include "Plane.h"
#include "zef_control.h"

using namespace AP;
using namespace AP_HAL;

void ZefControl::mult_mat(const float matrix[12][18], const double vector[12], double (&results)[12]) {
    int rows = 12;
    int cols = 18;
    for (int i = 0; i < rows; i++) {
        results[i] = 0.0;
        for (int j = 0; j < cols; j++) {
            results[i] += ((double)matrix[i][j]) * vector[j];
        }
    }
}

//this function uses only the pitch, roll and height values from the matrix
//it's only for manual flight, but with auto stabilization
void mult_mat_auto_stab(const float matrix[12][18], const double vector[12], double (&results)[12]) {
    int rows = 8;
    int cols = 18;
    for (int i = 0; i < rows; i++) {
        results[i] = 0.0;
        for (int j = 0; j < cols; j++) {
            bool can_use = true;
            switch(i) {
                case 0:
                    if(j==4||j==7) {
                        can_use = false;
                    }
                    break;
                case 1:
                    if(j==3||j==6) {
                        can_use = false;
                    }
                    break;
                case 3:
                    //if(j==2||j==3||j==6||j==11) {
                    if(j==3||j==6) {
                        can_use = false;
                    }
                    break;
                case 4:
                    if(j==4||j==7) {
                        can_use = false;
                    }
                    break;
                case 5:
                    if(j==3||j==6) {
                        can_use = false;
                    }
                    break;
                case 7:
                    //if(j==2||j==3||j==6||j==11) {
                    if(j==3||j==6) {
                        can_use = false;
                    }
                    break;
            }
            if (can_use == true) {
                results[i] += ((double)matrix[i][j]) * vector[j];
            }
        }
    }
}

void ZefControl::set_manual(int state) {
    is_manual_mode = state;
}

void ZefControl::set_RHO(double rho) {
    if(rho > 0.0) {
        RHO_air_density = rho;
    } else {
        RHO_air_density = 1.2;
    }
}
    
void ZefControl::find_matrix(const double longit_speed) {
    int max_options = sizeof(speed_refs);
    ref_index = max_options - 1;
    for (int i = max_options-1; i >= 0; i--) {
        if (longit_speed < speed_refs[i]) {
            ref_index = i;
        }
    }
    //ajusta o indice para ficar no range de k_matrix_all
    int len = sizeof(k_matrix_all) / sizeof(k_matrix_all[0]);
    if(ref_index > len-1) ref_index = len-1;
}

void ZefControl::add_traction(double longit_speed) {
    double Cd = 0.0;
    double Volume_2_3 = 9.0;
    for (int i = 0; i < 8; i++) {
        if(i % 2 == 0) {
            //double old_U = U_array[i];
            double f_static = 0.5*RHO_air_density*Cd*Volume_2_3*(powf(longit_speed,2)/4);
            U_eq[i] += f_static;
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%d >>> rho_density: %f --- longit_speed: %f --- Volume_2_3: %f", i, RHO_air_density, longit_speed, Volume_2_3);
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%d >>> old_U: %f --- new_U: %f --- f: %f", i, old_U, U_array[i], f_static);
        }
    }
    
    // I have proposed a method to add the antagonist force to the motors.
    // U_array[0] += antagonist_force;
    // U_array[2] += antagonist_force*-1;
    // U_array[4] += antagonist_force;
    // U_array[6] += antagonist_force*-1;

    /*# adicionar a tração para manter a posição (@carlos, favor confirmar se entra separado realmente)
    --> impares pois são a componente X... Y são pares
    U(1)=U(1)+1/2*rho*U^2/4
    U(3)=U(3)+1/2*rho*U^2/4
    U(5)=U(5)+1/2*rho*U^2/4
    U(7)=U(7)+1/2*rho*U^2/4*/
}

void ZefControl::adjust_for_manual(double (&U_array)[12]) {
    /*double ajuste_cal_roll_mot = 0.3;
    double ajuste_cal_pitch_mot = 0.6;
    double ajuste_cal_yaw_mot = 0.6;
    double ajuste_cal_cima_baixo = 0.6;
    double ajuste_cal_roll_estab = 0.3;
    double ajuste_cal_dir_esq = 0.3;
    double center_value = 0.0;*/
    
    double ajuste_cal_throttle = 30.0; //newtons
    double ajuste_cal_roll_mot = 0.3;
    double ajuste_cal_pitch_mot = 0.8;
    double ajuste_cal_yaw_mot = 0.8;
    double ajuste_cal_cima_baixo = 0.8;
    double ajuste_cal_roll_estab = 0.3;
    double ajuste_cal_dir_esq = 0.3;
    double center_value = 0.0;

	U_array[0] = J3_cmd_throttle*ajuste_cal_throttle + (ajuste_cal_pitch_mot*J2_cmd_pitch*ajuste_cal_throttle);
	U_array[1] = -ajuste_cal_dir_esq*RS_cmd_up_down*ajuste_cal_throttle;

	U_array[2] = J3_cmd_throttle*ajuste_cal_throttle - (ajuste_cal_yaw_mot*J4_cmd_yaw*ajuste_cal_throttle) - (ajuste_cal_throttle*ajuste_cal_pitch_mot*J2_cmd_pitch/2);
	U_array[3] = (ajuste_cal_cima_baixo*LS_cmd_letf_right*ajuste_cal_throttle) + (ajuste_cal_roll_mot*J1_cmd_roll*ajuste_cal_throttle);

	U_array[4] = J3_cmd_throttle*ajuste_cal_throttle - (ajuste_cal_pitch_mot*J2_cmd_pitch*ajuste_cal_throttle);
	U_array[5] = (ajuste_cal_dir_esq*RS_cmd_up_down*ajuste_cal_throttle) + (ajuste_cal_roll_mot*J1_cmd_roll*ajuste_cal_throttle);

	U_array[6] = J3_cmd_throttle*ajuste_cal_throttle + (ajuste_cal_yaw_mot*J4_cmd_yaw*ajuste_cal_throttle) - (ajuste_cal_throttle*ajuste_cal_pitch_mot*J2_cmd_pitch/2);
	U_array[7] = (-ajuste_cal_cima_baixo*LS_cmd_letf_right*ajuste_cal_throttle) + (ajuste_cal_roll_mot*J1_cmd_roll*ajuste_cal_throttle);

	U_array[8] = center_value + J4_cmd_yaw + (ajuste_cal_roll_estab*J1_cmd_roll); //(leme superior)
	U_array[9] = center_value + J2_cmd_pitch + (ajuste_cal_roll_estab*J1_cmd_roll); //(profundor direito)

	U_array[10] = center_value + (-J4_cmd_yaw) + (ajuste_cal_roll_estab*J1_cmd_roll); //(leme inferior)
	U_array[11] = center_value + (-J2_cmd_pitch) + (ajuste_cal_roll_estab*J1_cmd_roll); //(profundor esquerdo)

    /*for(int i = 0; i < 8; i+=2) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%d: %.2f , %.2f", i, U_array[i], U_array[i+1]);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=");*/

    /*
	# obter os comandos do radio controle
	J1 = obter o comando do stick do aileron
	J2 = obter o comando do stick do profundor
	J3 = obter o comando do stick do acelerador
	J4 = Obter o comando do stick do leme

	# Ganhos do controlador manual (pode entrar como variável global)
	define ab ajuste_cal_roll_mot, a2 ajuste_cal_pitch_mot, a4 ajuste_cal_yaw_mot, al ajuste_cal_cima_baixo, aa ajuste_cal_roll_estab, ar ajuste_cal_dir_esq

	U(1) = J3 + ajuste_cal_pitch_mot*J2
	U(2) = ajuste_cal_dir_esq*RS + ajuste_cal_roll_mot*J1
	U(3) = J3 - ajuste_cal_yaw_mot*J4
	U(4) = ajuste_cal_cima_baixo*LS + ajuste_cal_roll_mot*J1
	U(5) = J3 - ajuste_cal_pitch_mot*J2
	U(6) = ajuste_cal_dir_esq*RS + ajuste_cal_roll_mot*J1
	U(7) = J3 + ajuste_cal_yaw_mot*J4
	U(8) = ajuste_cal_cima_baixo*LS + ajuste_cal_roll_mot*J1
	U(9) = J4 + ajuste_cal_roll_estab*J1 (leme superior)
	U(10) = J2 + ajuste_cal_roll_estab*J1 (profundor direito)
	U(11) = -J4 + ajuste_cal_roll_estab*J1 (leme inferior)
	U(12) = -J2 + ajuste_cal_roll_estab*J1 (profundor esquerdo)*/
}

void ZefControl::manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch) {
    double multiplicador_comandos = 1.0; //4500;
    double multiplicador_motor = 1.0;
    J1_cmd_roll = aileron*multiplicador_comandos;
    J2_cmd_pitch = elevator*multiplicador_comandos;
    J3_cmd_throttle = throttle*multiplicador_motor;
    J4_cmd_yaw = rudder*multiplicador_comandos;
    RS_cmd_up_down = right_switch;
    LS_cmd_letf_right = left_switch;

    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J1: %f", J1_cmd_roll);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J2: %f", J2_cmd_pitch);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J3: %f", J3_cmd_throttle);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "J4: %f", J4_cmd_yaw);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RS: %f", RS_cmd_up_down);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LS: %f", LS_cmd_letf_right);*/
}

void ZefControl::antagonist_force_update() {
    U_eq[0] = U_eq[0] - antagonist_force;
    U_eq[2] = U_eq[2] + antagonist_force;
    U_eq[4] = U_eq[4] - antagonist_force;
    U_eq[6] = U_eq[6] + antagonist_force;
}

void ZefControl::print_output_data() {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1: %f -- ang: %f", F1_forca_motor, d1_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2: %f -- ang: %f", F2_forca_motor, d2_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3: %f -- ang: %f", F3_forca_motor, d3_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4: %f -- ang: %f", F4_forca_motor, d4_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang v cima: %f -- ang v baixo: %f", dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang h dir: %f -- ang h esq: %f", dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo);
}

double sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

double min(double x, double y) {
    if(x < y) return x;
    if(y < x) return y;
    return x;
}

void ZefControl::update(
        double eU, double eV, double eW,
        double eP, double eQ, double eR,
        double eroll, double epitch, double eyaw,
        double eX, double eY, double eZ) {

    double vetor_erro[18] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double E[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

    for (int i=0; i < 12; i++) {
        U_prop_att[i] = 0.0;
        U_int_att[i] = 0.0;

        U_prop_pos[i] = 0.0;
        U_int_pos[i] = 0.0;

        U_prop_alt[i] = 0.0;
        U_int_alt[i] = 0.0;

        U_eq[i] = 0.0;

        U[i] = 0.0;

    }


    if(is_manual_mode == 1) {
        adjust_for_manual(U);
    } else {
        vetor_erro[0] = eU;
        vetor_erro[1] = eV;
        vetor_erro[2] = eW;
        vetor_erro[3] = eP;
        vetor_erro[4] = eQ;
        vetor_erro[5] = eR;
        vetor_erro[6] = eroll;
        vetor_erro[7] = epitch;
        vetor_erro[8] = eyaw;
        vetor_erro[9] = eX;
        vetor_erro[10] = eY;
        vetor_erro[11] = eZ;

        for(int i = 12; i < 18; i++) {
            vetor_erro[i] = 0.0;
        }

        E[0] = eroll;
        E[1] = epitch;
        E[2] = eyaw;
        E[3] = eX;
        E[4] = eY;
        E[5] = eZ;
    };

    uint32_t now_ms = AP_HAL::millis();
    float dt = 0.01f; // Just to initialize, will be updated below
    
    if (_last_update_ms != 0) {
        dt = (now_ms - _last_update_ms) * 0.001f; // Convert ms to seconds
    }
    _last_update_ms = now_ms;

    for (int i = 0; i < 6; i++) {
        // Trapezoidal integration: Q[i] += (dt/2) * (E[i] + E_prev[i])
        Q[i] += (dt / 2.0f) * (E[i] + E_prev[i]);
        if (i == 0 ){
            Q[i] = constrain_float(Q[i], -2.0f, 2.0f);  
        }
        if (i == 1 ){
            Q[i] = constrain_float(Q[i], -4.0f, 4.0f);  
        }
        Q[i] = constrain_float(Q[i], -8.0f, 8.0f);  
        E_prev[i] = E[i];  
    }

    vetor_erro[12] = Q[0]; // Roll
    vetor_erro[13] = Q[1]; // Pitch
    vetor_erro[14] = Q[2]; // Yaw
    vetor_erro[15] = Q[3]; // x_position_n_s
    vetor_erro[16] = Q[4]; // y_position_e_w
    vetor_erro[17] = Q[5]; // z_position_height

    // mult_mat(k_matrix_all[14], vetor_erro, U); Elevator
    if (is_manual_mode == 2) {
        mult_mat_auto_stab(k_matrix_all[current_matrix_index], vetor_erro, U);
    } else {
        Attitude_Proportional_Control(vetor_erro);
        Attitude_Integral_Control(vetor_erro);

        Position_Proportional_Control(vetor_erro);
        Position_Integral_Control(vetor_erro);
        
        Altitude_Proportional_Control(vetor_erro);
        Altitude_Integral_Control(vetor_erro);

        for (int i = 0; i < 12; i++) {
            U[i] = U_prop_att[i] + U_int_att[i] + U_prop_pos[i] + U_int_pos[i] + U_prop_alt[i] + U_int_alt[i];
        }
    }
    
    //memcpy(last_X, X, sizeof(last_X));
    // add_traction(U, U_longit_speed);

    antagonist_force_update();

    for (int i = 0; i < 12; i++) {
        U[i] += U_eq[i];
    }

    if (is_manual_mode == 2) {
        //modo hibrido
        double nU[12];
        adjust_for_manual(nU);
        for ( int i = 0; i < 12; i++) {
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "values 1 - %f --- 2 - %f", U[i], nU[i]);
            U[i] += nU[i];
        }
    }
    
}

void ZefControl::Attitude_Proportional_Control(double vector_error[18]) {

    double Att_error[18] = {0.0, 0.0, 0.0, 
        vector_error[3], vector_error[4], vector_error[5], 
        vector_error[6], vector_error[7], vector_error[8], 
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0};

    mult_mat(k_matrix_all[current_matrix_index], Att_error, U_prop_att);
}

void ZefControl::Attitude_Integral_Control(double vector_error[18]) {

    double Att_error[18] = {0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            vector_error[12], vector_error[13], vector_error[14], 
                            0.0, 0.0, 0.0};

    mult_mat(k_matrix_all[current_matrix_index], Att_error, U_int_att);
}

void ZefControl::Altitude_Proportional_Control(double vector_error[18]) {
    double Alt_error[18] = {0.0, 0.0, vector_error[2], 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, vector_error[11], 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0};

    mult_mat(k_matrix_all[current_matrix_index], Alt_error, U_prop_alt);
}

void ZefControl::Altitude_Integral_Control(double vector_error[18]) {
    double Alt_error[18] = {0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, vector_error[17]};

    mult_mat(k_matrix_all[current_matrix_index], Alt_error, U_int_alt);
}

void ZefControl::Position_Proportional_Control(double vector_error[18]) {
    double Pos_error[18] = {vector_error[0], vector_error[1], 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0,
                            vector_error[9], vector_error[10], 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0};

    mult_mat(k_matrix_all[current_matrix_index], Pos_error, U_prop_pos);
}

void ZefControl::Position_Integral_Control(double vector_error[18]) {
    double Pos_error[18] = {0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            vector_error[15], vector_error[16], 0.0};

    mult_mat(k_matrix_all[current_matrix_index], Pos_error, U_int_pos);
}   

double ZefControl::set_angle_range(double last_angle, double v1, double v0) {
    double pi = 3.14159265358979311600;
    double Ang_max = 210/180*pi;
    double d_angulo_motor_ant = last_angle;
    double d_angulo_motor = atan2f(v1, v0);
    if (d_angulo_motor>(2*pi-Ang_max) && d_angulo_motor_ant < -(2*pi-Ang_max) ) {
        d_angulo_motor =  -(2*pi-d_angulo_motor);
    }
    if (d_angulo_motor<-(2*pi-Ang_max) && d_angulo_motor_ant > (2*pi-Ang_max) ) {
        d_angulo_motor =  (2*pi-d_angulo_motor);
    }
    return d_angulo_motor;
}

double ZefControl::get_engine_command(double motor_force) {
    double command = 0.0;
    if( motor_force >= F_min_mot ) {
        if (motor_force > F_max_mot) {
            command = comando_maximo;
        } else {
            command =  (coeficiente_quadratico_forca * powf(motor_force, 2)) + (coeficiente_linear_forca * motor_force) + coeficiente_nulo_forca;
        }
    }

    return command;
}

void ZefControl::set_power_and_angles() {
	double ang_anterior = 0.0;
    double coef_antec_mov = 0.5;
    double f_min_servo = F_min_mot * coef_antec_mov;

    // Encontra a força dos motores absoluta e o angulo dos motores
    F1_forca_motor = sqrtf(powf(U[0], 2) + powf(U[1], 2));
    if( F1_forca_motor >= f_min_servo ) {
        ang_anterior = d1_angulo_motor;
        d1_angulo_motor = set_angle_range(ang_anterior, U[1], U[0]);
    }

    F2_forca_motor = sqrtf(powf(U[2], 2) + powf(U[3], 2));
    if( F2_forca_motor >= f_min_servo ) {
        ang_anterior = d2_angulo_motor;
        d2_angulo_motor = set_angle_range(ang_anterior, U[3], U[2]);
    }

    F3_forca_motor = sqrtf(powf(U[4], 2) + powf(U[5], 2));
    if( F3_forca_motor >= f_min_servo ) {
        ang_anterior = d3_angulo_motor;
        d3_angulo_motor = set_angle_range(ang_anterior, U[5], U[4]);
    }

    F4_forca_motor = sqrtf(powf(U[6], 2) + powf(U[7], 2));
    if( F4_forca_motor >= f_min_servo ) {
        ang_anterior = d4_angulo_motor;
        d4_angulo_motor = set_angle_range(ang_anterior, U[7], U[6]);
    }

    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[0], U[1], powf(U[0], 2), powf(U[1], 2), F1_forca_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[2], U[3], powf(U[2], 2), powf(U[3], 2), F2_forca_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[4], U[5], powf(U[4], 2), powf(U[5], 2), F3_forca_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4 - v1 %f - v2 %f - p1 %f - p2 %f - raiz: %f", U[6], U[7], powf(U[6], 2), powf(U[7], 2), F4_forca_motor);*/

    dvu_ang_estab_vert_cima = U[8];
    dvd_ang_estab_vert_baixo = U[9];

    dhr_ang_estab_horiz_direito = U[10];
    dhl_ang_estab_horiz_esquerdo = U[11];

    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F1: %f -- ang: %f", F1_forca_motor, d1_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F2: %f -- ang: %f", F2_forca_motor, d2_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F3: %f -- ang: %f", F3_forca_motor, d3_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "F4: %f -- ang: %f", F4_forca_motor, d4_angulo_motor);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");*/
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang v cima: %f -- ang v baixo: %f", dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ang h dir: %f -- ang h esq: %f", dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo);*/
}

void ZefControl::put_forces_in_range() {
    /*
    Encontra o comando necessário para gerar a força desejada
    com base em uma função quadrática de ajuste.
    Se a força for menor que a mínima, zera e se for maior que a máxima, limita pela máxima.
    */

    comando_M1 = get_engine_command(F1_forca_motor);
    comando_M2 = get_engine_command(F2_forca_motor);
    comando_M3 = get_engine_command(F3_forca_motor);
    comando_M4 = get_engine_command(F4_forca_motor);
}

int ZefControl::get_value_to_pwm_servo(double in_value, int min_pwm, int max_pwm) {
    int range = (max_pwm - min_pwm)/2;
    int center_value = (min_pwm + max_pwm)/2;
    int retVal = ((double)(in_value/3.14159265) * range) + center_value;
    if(retVal > max_pwm) retVal = max_pwm;
    if(retVal < min_pwm) retVal = min_pwm;
    return retVal;
}

int ZefControl::get_value_to_pwm_motor(double in_value, int min_pwm, int max_pwm) {
    int range = (max_pwm - min_pwm);
    int retVal = (in_value * range) + min_pwm;
    if(retVal > max_pwm) retVal = max_pwm;
    if(retVal < min_pwm) retVal = min_pwm;
    return retVal;
}

float ZefControl::dead_zone(double input_value, double dead_zone_limit) {
    double retVal = input_value;
    if((input_value < dead_zone_limit) && (input_value > -dead_zone_limit)) retVal = 0.0;
    return retVal;
}

Vector3f ZefControl::rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector) {
    // Criar um quaternion a partir dos ângulos de Euler (roll, pitch, yaw)
    Quaternion q;
    q.from_euler(roll, pitch, yaw);

    // Rotacionar o vetor do referencial inercial para o referencial da aeronave
    Vector3f body_vector = inertial_vector;
    q.earth_to_body(body_vector);

    return body_vector;
}

void ZefControl::getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]) {
    double earth_radius = 6.371e6;
    double convert_degrees_to_radians = 0.01745329252;
    double adjust_gps_reading = 1e-7;
    // pi/180
    /*GCS_SEND_TEXT(MAV_SEVERITY_INFO, "d_lat: %d - d_lon %d", (int)desired_position_lat, (int)desired_position_long);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "c_lat: %d - c_lon %d", (int)current_position_lat, (int)current_position_long);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=====");*/

    double error_x_earth_ref = convert_degrees_to_radians * (desired_position_lat - current_position_lat) * adjust_gps_reading * earth_radius;
    double error_y_earth_ref = convert_degrees_to_radians * (desired_position_long - current_position_long) * adjust_gps_reading * earth_radius * cosf (convert_degrees_to_radians * current_position_lat);
    double error_z_earth_ref = -(desired_position_alt - current_position_alt); //*0.01;
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "x: %f -- y: %f -- z: %f", error_x_earth_ref, error_y_earth_ref, error_z_earth_ref);

    // Negativo por causa do referencial North-East-Down (NED)
    double error_x_airship_ref = sinf(azimute) * error_x_earth_ref + cosf(azimute) *  error_y_earth_ref;
    double error_y_airship_ref = cosf(azimute) * error_x_earth_ref - sinf(azimute) *  error_y_earth_ref;
    double error_z_airship_ref = error_z_earth_ref;
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>> x: %f -- y: %f -- z: %f", error_x_airship_ref, error_y_airship_ref, error_z_airship_ref);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, ">>> z: %f", current_position_alt);

    ret_errors[0] = error_x_airship_ref;
    ret_errors[1] = error_y_airship_ref;
    ret_errors[2] = error_z_airship_ref;
    
    //update the error ref position here
    X_ref[11] = desired_position_alt;
}

void ZefControl::set_position_reference(double x_ref, double y_ref, double z_ref) {
    X_ref[9] = x_ref;   // x_position_n_s
    X_ref[10] = y_ref;  // y_position_e_w  
    X_ref[11] = z_ref;  // z_position_height
}

void ZefControl::set_roll_reference(double roll_ref) {
    X_ref[6] = roll_ref; 
}

void ZefControl::set_matrix_index(int index) {
    current_matrix_index = index;
}

int ZefControl::get_matrix_index() const {
    return current_matrix_index;
}

void ZefControl::set_accumulative_error() {
    for (int i = 0; i < 6; i++) {
        Q[i] = 0.0;
    }
}

ZefControl* ZefControl::instancePtr = nullptr;
