#include <identification.h>
#include <drivers.h>
#include <fmath.h>
#include <lqr.h>

#define LED_GPIO        TGPIOE
#define LED_PIN         2


#define SAMPLES_COUNT   ((uint32_t)4000)


void line_stabilise()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    LQR<1, 2> lqr; 
 
    int dt = 4;

    //float k[2]  = {0.2, 2.0};
    //float ki[2] = {0.0, 0.0};
    
    //float k[2]  = {0.17224, 3.57063};
    //float ki[2] = {0.0,  30.0};

    float k[2]  = {1.8,   0.8};
    float ki[2] = {0.0,   0.0};

    float speed = 0.0; 
    
    lqr.init(k, ki, 1.0, dt/1000.0);

    uint32_t  on_line_time = timer.get_time();

    while (1)
    {
        led = 1;
        float angular_rate  = line_sensor.angular_rate;
        float angle         = line_sensor.angle;  
        
        lqr.x[0]  = angular_rate;
        lqr.x[1]  = angle;

        lqr.xr[0] = 0.0;
        lqr.xr[1] = 0.0;

        lqr.step();

        float u = lqr.u[0];
        
        //terminal << "lqr = " <<  lqr.error_sum[1]*lqr.ki[1]  << " " <<  u << "\n";

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
            on_line_time = timer.get_time();
            if (speed < 1.0)
            {
                speed = speed + 0.01;
            } 
        }
        else
        {
            speed = 0.15;
        }
        
        
        if (timer.get_time() > on_line_time + 100)
        {
            motor_control.set_torque(0, 0);
        }
        else
        {
            motor_control.set_torque((speed+u)*MOTOR_CONTROL_MAX, (speed-u)*MOTOR_CONTROL_MAX);
        }

        led = 0;

        timer.delay_ms(dt);
    }

}


void turn_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    float result_log[SAMPLES_COUNT][3];
    
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
       result_log[n][0] = 0;
       result_log[n][1] = 0;
       result_log[n][2] = 0; 
    }

    timer.delay_ms(500);

    led = 1;

    //move robot forward to line
    motor_control.set_torque(0.15*MOTOR_CONTROL_MAX, 0.15*MOTOR_CONTROL_MAX);
    timer.delay_ms(200);
    motor_control.set_torque(0, 0);
    timer.delay_ms(200);

    //align robot to center
    for (unsigned int i = 0; i < 200; i++)
    {
        float angle = line_sensor.angle;
        float e = 0.0 - angle;
        float forward = 0.05;
        float turn    = 2.0*e; 
        
        motor_control.set_torque((forward + turn)*MOTOR_CONTROL_MAX, (forward - turn)*MOTOR_CONTROL_MAX);
        timer.delay_ms(4);
    }

    motor_control.set_torque(0, 0);

    led = 0;
    timer.delay_ms(200);

    
    uint32_t state      = 0;
    uint32_t time_next  = 0;

    float turn_max      = 0.12;
    float angle_max     = 0.05;

    float turn          = turn_max;

    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {   
        float angle = line_sensor.angle;
        float angular_rate = line_sensor.angular_rate;

        if (state == 0 && angle > angle_max)
        {
            turn  = 0;
            time_next = timer.get_time() + 250;
            state = 1;
        } 
        else if (state == 1 && timer.get_time() > time_next)
        {
            turn  = -turn_max;
            state = 2;
        }
        else if (state == 2 && angle < -angle_max)
        {
            turn  = 0;
            time_next = timer.get_time() + 250;
            state = 3;
        }
        else if (state == 3 && timer.get_time() > time_next)
        {
            turn  = turn_max;
            state = 0;
        }
    
        motor_control.set_torque((turn)*MOTOR_CONTROL_MAX, (-turn)*MOTOR_CONTROL_MAX);
        timer.delay_ms(4);

        result_log[n][0] = turn;
        result_log[n][1] = angular_rate;
        result_log[n][2] = angle; 
    }

    motor_control.set_torque(0, 0);
    timer.delay_ms(100);
    
    terminal << "\n\n\n";
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
        terminal << n << " " << result_log[n][0] << " " << result_log[n][1] << " "<< result_log[n][2] << "\n";
    }
    terminal << "\n\n\n";
    
}
