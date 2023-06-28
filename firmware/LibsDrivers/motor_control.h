#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>
#include <as5600_t.h>


#define MOTOR_CONTROL_MAX       ((int32_t)1024)

#define MOTOR_POLES             ((int32_t)14)
#define MOTOR_KV                ((int32_t)250)
#define MOTOR_CURRENT_MAX       ((int32_t)5)



class MotorControl
{
    public:
        void init();
        void set_torque(int32_t left_torque, int32_t right_torque);

        void callback(); 

        void hold();

    private:
        void    set_torque_from_rotation(int32_t torque, int32_t phase, uint32_t rotor_angle, int motor_id);
        int32_t clamp(int32_t value, int32_t min, int32_t max);

    private: 
        int32_t left_torque;
        int32_t right_torque; 

        PWMLeft     left_pwm;
        PWMRight    right_pwm;

    public:
        AS5600T<TGPIOD, 14, 15, 2> left_encoder;
        AS5600T<TGPIOE,  0,  1, 2> right_encoder;

};

#endif