#include <identification.h>
#include <drivers.h>
#include <fmath.h>
#include <shaper.h>
#include <lqr_single.h>


#define LED_GPIO        TGPIOE
#define LED_PIN         2


#define SAMPLES_COUNT   ((uint32_t)3000)


void motor_identification()
{
    motor_control.set_torque(0, 0); 
    timer.delay_ms(200); 

    //run motor on full speed
    motor_control.set_torque(MOTOR_CONTROL_MAX, 0); 
    timer.delay_ms(800);
    unsigned int steps_max = 400;

    //motor free run speed, in RPM
    float motor_max_speed_mean = 0;
    float motor_max_speed_var  = 0;

    //average maximum speed
    for (unsigned int i = 0; i < steps_max; i++)
    {
        float speed = motor_control.get_left_velocity()*60.0/(2.0*PI);
        motor_max_speed_mean+= speed;
        timer.delay_ms(2);
    } 

    motor_max_speed_mean = motor_max_speed_mean/steps_max;

    //encoder velocity measurement variance
    for (unsigned int i = 0; i < steps_max; i++)
    {
        float speed = motor_control.get_left_velocity()*60.0/(2.0*PI);
        float dif   = speed - motor_max_speed_mean;
        motor_max_speed_var+= dif*dif;
        timer.delay_ms(2); 
    } 

    motor_max_speed_var = motor_max_speed_var/steps_max;

    motor_control.set_torque(0, 0); 
    timer.delay_ms(200);

    //print results
    terminal << "motor_max_speed_mean   " << motor_max_speed_mean << " rpm\n";
    terminal << "motor_max_speed_var    " << motor_max_speed_var << " rpm^2\n";

    //time constant estimation
    steps_max = 20;

    float time_constant = 0.0;

    for (unsigned int i = 0; i < steps_max; i++)
    {
        int32_t time_start = timer.get_time();

        motor_control.set_torque(MOTOR_CONTROL_MAX, 0); 

        while (motor_control.get_left_velocity()*(60.0/(2.0*PI)) < 0.632*motor_max_speed_mean)
        {
            __asm("nop");
        }

        int32_t time_stop = timer.get_time();

        motor_control.set_torque(0, 0); 
        timer.delay_ms(100);

        int32_t dt = time_stop - time_start; 

        time_constant+= dt;
    }

    time_constant = time_constant/steps_max;

    terminal << "time_constant   " << time_constant << " ms\n";
    terminal << "\n\n";
}




void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //float left_required_positions[]  = {0.0,  1.0, -1.0, 0.0,  0.0, 1.0, -1.0, 0.0};
    //float right_required_positions[] = {0.0,  0.0,  0.0, 1.0, -1.0, -1.0, 1.0, 0.0};

    float left_required_positions[]  = {0.0, 1.0, 0.0, -1.0,   0.0, -1.0, 0.0,  1.0,   0.0, 1.0, 0.0, -1.0,   0.0, 0.0, 0.0,  0.0};
    float right_required_positions[] = {0.0, 1.0, 0.0, -1.0,   0.0,  1.0, 0.0, -1.0,   0.0, 0.0, 0.0, 0.0,    0.0, 1.0, 0.0, -1.0};

    float speed_max = 1500.0*2.0*PI/60.0;

    //float k  = 10.0;  
    //float ki = 0.4;         

    float k  = 5.0;   
    float ki = 0.2;         


    LQRSingle left_lqr, right_lqr;

    left_lqr.init(k, ki, speed_max);
    right_lqr.init(k, ki, speed_max);

    Shaper shaper_left, shaper_right;  

    shaper_left.init(0.5, -2.0); 
    shaper_right.init(0.5, -2.0); 

 
    unsigned int steps = 0; 

    int32_t dt = 4;
    float curr_dt_fil = 0.0;

    while (1) 
    {    
        int32_t time_start = timer.get_time();

        //obtain required value
        unsigned int idx = (steps/200)%16;   
        float left_required  = left_required_positions[idx]*PI*0.75;
        float right_required = right_required_positions[idx]*PI*0.75;
 
        //obtain measured
        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        //LQR controller
        float left_u   = left_lqr.step(left_required, left_position);
        float right_u  = right_lqr.step(right_required, right_position);
  
        //output shaping
        float vl_shaped = shaper_left.step(left_u); 
        float vr_shaped = shaper_right.step(right_u);

        //send to motors
        motor_control.set_velocity(vl_shaped, vr_shaped);

        terminal << steps << " " << left_u << " " << right_u << " " << left_position << " " << right_position << " " << curr_dt_fil << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        int32_t dif = time_stop - time_start;
        int32_t curr_dt = dt - dif;

        curr_dt_fil = 0.9*curr_dt_fil + 0.1*dif;

        if (curr_dt > 0)
        {
            timer.delay_ms(curr_dt);
        }
    }
}
