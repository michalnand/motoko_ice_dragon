#include <identification.h>
#include <drivers.h>
#include <fmath.h>
#include <shaper_s.h>
#include <lqr_single.h>

//#include <position_control_lqr.h>
#include <position_control_lqg.h>


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

    //dims in mm
    float wheel_diameter = 34.0;
    float wheel_brace    = 80.0;


    //max speed, 1500rpm
    float speed_max     = 1500*2.0*PI/60.0;

    float required_forward[] = {0.0, 200.0, 0.0, -200.0, 0.0,  0.0,  0.0,   0.0,  0.0};
    float required_turn[]    = {0.0, 0.0,  0.0,  0.0,  0.0, 90.0, 0.0, -90.0, 0.0};

    //float required_forward[] = {0.0, 0.0, 0.0, 0.0};  
    //float required_turn[]    = {0.0, 90.0, 0.0, -90.0};

    //controllers
    float pf = 0.02;  
    float df = 0.6;

    float pt = 0.4;  
    float dt = 0.8;  

    ShaperS shaper_forward, shaper_turn;
    
    //shaper init
    shaper_forward.init(0.1, 0.1, 0.0025, 0.0025);   
    shaper_turn.init(0.1, 0.1, 0.25, 0.25);             

    uint32_t steps = 0;

    float e0_forward = 0.0;
    float e1_forward = 0.0;

    float e0_turn = 0.0;
    float e1_turn = 0.0;     
 
    while (1)  
    {     
        uint32_t idx = (steps/500)%9;    

        //required values
        float req_forward = required_forward[idx];
        float req_turn    = required_turn[idx]*PI/180.0;

        //obtain current state  

        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.25*(right_position + left_position)*wheel_diameter;
        float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

        //compute controller
        e1_forward = e0_forward;
        e0_forward = req_forward - distance;

        e1_turn = e0_turn;
        e0_turn = req_turn - angle;

        float u_forward = pf*e0_forward + df*(e0_forward - e1_forward);
        float u_turn    = pt*e0_turn + dt*(e0_turn - e1_turn); 

        float u_forward_sh = shaper_forward.step(u_forward);
        float u_turn_sh    = shaper_turn.step(u_turn);

        float u_left  = u_forward_sh - u_turn_sh;
        float u_right = u_forward_sh + u_turn_sh;    

        u_left  = clip(u_left, -1.0, 1.0);
        u_right = clip(u_right, -1.0, 1.0); 

        motor_control.set_velocity(u_left*speed_max, u_right*speed_max);

        terminal << steps << " " << u_forward << " " << u_turn << " " << distance << " " <<  angle << "\n";
        steps++;

        timer.delay_ms(4);
    }
}
