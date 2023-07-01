#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>
#include <as5600_t.h>


//set_torque min and max range
#define MOTOR_CONTROL_MAX       ((int32_t)1024)

//dt step in microseconds, 2kHz, 500uS
#define MOTOR_CONTROL_DT    ((uint32_t)500)


#define MOTOR_POLES             ((int32_t)14)


class MotorControl
{
    public:
        void init();
        void set_torque(int32_t left_torque, int32_t right_torque);

        void callback(); 

        void hold();

        //return raw angle from encoder, 0..4095
        int32_t get_left_angle();
        
        //wheel position (angle), 1.0 is equal to one full forward rotation, -1.0 for backward
        float get_left_position();
        
        //wheel angular velocity, 1.0 is equal to one full forward rotation per second, -1.0 for backward
        float get_left_velocity();

        //return raw angle from encoder, 0..4095
        int32_t get_right_angle();

        //wheel position (angle), 1.0 is equal to one full forward rotation, -1.0 for backward
        float get_right_position();

        //wheel angular velocity, 1.0 is equal to one full forward rotation per second, -1.0 for backward
        float get_right_velocity();

    private:
        void    set_torque_from_rotation(int32_t torque, int32_t phase, uint32_t rotor_angle, int motor_id);
        int32_t clamp(int32_t value, int32_t min, int32_t max);

    private: 
        int32_t left_torque;
        int32_t right_torque; 

        PWMLeft     left_pwm;
        PWMRight    right_pwm;

    private:
        AS5600T<TGPIOD, 14, 15, 2> left_encoder;
        AS5600T<TGPIOE,  0,  1, 2> right_encoder;
};

#endif