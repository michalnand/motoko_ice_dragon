#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>
#include <as5600_t.h>
//#include <lqr_single.h>
#include <lqg_single.h>

//set_torque min and max range
#define MOTOR_CONTROL_MAX       ((int32_t)1024) 

//dt step in microseconds, 4kHz, 250uS
#define MOTOR_CONTROL_DT    ((uint32_t)250)




#define MOTOR_POLES             ((uint32_t)14)


class MotorControl
{
    public:
        void init();

        //helping fuction for testing, only when running in open loop
        void set_torque(float left_torque, float right_torque);

        //required velocity, in rad/s
        void set_velocity(float left_velocity, float right_velocity);

        void callback_torque(); 

        void hold();

        //return raw angle from encoder, 0..4095
        int32_t get_left_angle();
        
        //wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_left_position();
        float get_left_position_fil();  
        
        //wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_left_velocity();

        //return raw angle from encoder, 0..4095
        int32_t get_right_angle();
        
        //wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_right_position();
        float get_right_position_fil();

        //wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_right_velocity();

    private: 
        //void    set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, int motor_id);
        void    set_torque_from_rotation(int32_t torque, bool brake, uint32_t rotor_angle, int motor_id);
        int32_t clamp(int32_t value, int32_t min, int32_t max);
        int32_t shrink(int32_t value, int32_t shrink_value);

    private: 
        int32_t left_torque;
        int32_t right_torque;

        float left_req_velocity;
        float right_req_velocity;

        PWMLeft     left_pwm;
        PWMRight    right_pwm;


        uint32_t steps;

    private:
        AS5600T<TGPIOD, 15, 14, 2> left_encoder;
        AS5600T<TGPIOE,  0,  1, 2> right_encoder;

        /*
        LQRSingle left_controller;
        LQRSingle right_controller;
        */

       
        LQGSingle left_controller;
        LQGSingle right_controller;
};

#endif