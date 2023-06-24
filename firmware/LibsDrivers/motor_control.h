#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>
#include <as5600.h>


#define MOTOR_CONTROL_MAX       ((int32_t)1024)

#define MOTOR_POLES             ((int32_t)14)
#define MOTOR_KV                ((int32_t)250)
#define MOTOR_CURRENT_MAX       ((int32_t)5)

//maximal motor torque x 4096
#define MOTOR_TORQUE_MAX        ((4096*MOTOR_CURRENT_MAX)/MOTOR_KV)



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

        TI2C<TGPIOD, 14, 15, 5> left_i2c;
        TI2C<TGPIOE, 0,   1, 5> right_i2c;


        PWMLeft     left_pwm;
        PWMRight    right_pwm;

    public:
        AS5600      left_encoder;
        AS5600      right_encoder;
};

#endif