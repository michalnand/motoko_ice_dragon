#include <identification.h>
#include <drivers.h>
#include <fmath.h>
#include <lqr.h>

#define LED_GPIO        TGPIOE
#define LED_PIN         2


#define SAMPLES_COUNT   ((uint32_t)3000)

void motor_identification()
{
    motor_control.set_torque(0, 0); 
    timer.delay_ms(100); 

    motor_control.set_torque(MOTOR_CONTROL_MAX, 0); 
    timer.delay_ms(800);

    unsigned int steps_max = 400;

    //motor free run speed, in RPM
    float motor_max_speed_mean = 0;
    float motor_max_speed_var  = 0;

    for (unsigned int i = 0; i < steps_max; i++)
    {
        float speed = motor_control.get_left_velocity()*60.0/(2.0*PI);
        motor_max_speed_mean+= speed;
        timer.delay_ms(2);
    } 

    motor_max_speed_mean = motor_max_speed_mean/steps_max;

    for (unsigned int i = 0; i < steps_max; i++)
    {
        float speed = motor_control.get_left_velocity()*60.0/(2.0*PI);
        float dif   = speed - motor_max_speed_mean;
        motor_max_speed_var+= dif*dif;
        timer.delay_ms(2); 
    } 

    motor_max_speed_var = motor_max_speed_var/steps_max;



    motor_control.set_torque(0, 0); 
    timer.delay_ms(800);

    terminal << "motor_max_speed_mean   " << motor_max_speed_mean << " rpm\n";
    terminal << "motor_max_speed_var    " << motor_max_speed_var << " rpm^2\n";


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

    terminal << "time_constant   " << dt   << " ms\n";
    terminal << "\n\n";
}


void turn_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    float speed_min =  50.0*2.0*PI/60.0;  //RPM to rad/s
    float speed_max = 500.0*2.0*PI/60.0;  //RPM to rad/s
    float angle_max = 40*PI/180.0;        //degrees to rad

    float speed     = speed_min;
    
    uint32_t state      = 0;
    uint32_t time_next  = 0;  


    float result_log[SAMPLES_COUNT][3];
    
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
       result_log[n][0] = 0; 
       result_log[n][1] = 0;
       result_log[n][2] = 0; 
    }
    
    


    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {   
        float angle         = gyro_sensor.angle;
        float angular_rate  = gyro_sensor.angular_rate;

        float k             = (float)n/(float)SAMPLES_COUNT;
        float use_speed     = (1.0 - k)*speed_min + k*speed_max;

        if (state == 0 && angle > angle_max)
        {
            speed  = 0;
            time_next = timer.get_time() + 250;
            state = 1;
        } 
        else if (state == 1 && timer.get_time() > time_next)
        {
            speed  = -use_speed;
            state = 2;
        }
        else if (state == 2 && angle < -angle_max)
        {
            speed  = 0;
            time_next = timer.get_time() + 250;
            state = 3;
        }
        else if (state == 3 && timer.get_time() > time_next)
        {
            speed  = use_speed;
            state = 0;
        }
    
        motor_control.set_velocity(-speed, speed);
        timer.delay_ms(4); 

        result_log[n][0] = speed;
        result_log[n][1] = angular_rate;
        result_log[n][2] = angle; 
    }


    motor_control.set_velocity(0, 0); 
    timer.delay_ms(100);
    
    terminal << "\n\n\n"; 
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
        terminal << n << " " << result_log[n][0] << " " << result_log[n][1] << " " << result_log[n][2]  << "\n";
    }
    terminal << "\n\n\n";
}







void forward_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    float speed_min =  50.0*2.0*PI/60.0;  //RPM to rad/s
    float speed_max = 300.0*2.0*PI/60.0;  //RPM to rad/s


    float wheel_diameter= 32.0;             //wheel diameter in mm
    float distance_max  = 100.0;            //distance to travel in mm
    distance_max        = (2.0*PI)*distance_max/(wheel_diameter*PI);  //mm to radians


    float speed     = speed_min;
    
    uint32_t state      = 0;
    uint32_t time_next  = 0;  


    float result_log[SAMPLES_COUNT][3];
    
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
       result_log[n][0] = 0; 
       result_log[n][1] = 0;
       result_log[n][2] = 0; 
    }
    
    


    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {   
        float distance      = (motor_control.get_left_position() + motor_control.get_right_position())/2.0;
        float velocity      = (motor_control.get_left_velocity() + motor_control.get_right_velocity())/2.0;

        float k             = (float)n/(float)SAMPLES_COUNT;
        float use_speed     = (1.0 - k)*speed_min + k*speed_max;

        if (state == 0 && distance > distance_max)
        { 
            speed  = 0;
            time_next = timer.get_time() + 250;
            state = 1;
        }  
        else if (state == 1 && timer.get_time() > time_next)
        {
            speed  = -use_speed;
            state = 2;
        }
        else if (state == 2 && distance < -distance_max)
        {
            speed  = 0;
            time_next = timer.get_time() + 250;
            state = 3;
        }
        else if (state == 3 && timer.get_time() > time_next)
        {
            speed  = use_speed;
            state = 0; 
        }
    
        motor_control.set_velocity(speed, speed);
        timer.delay_ms(4); 

        result_log[n][0] = speed;
        result_log[n][1] = velocity;
        result_log[n][2] = distance; 
    }


    motor_control.set_velocity(0, 0); 
    timer.delay_ms(100);
    
    terminal << "\n\n\n"; 
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
        terminal << n << " " << result_log[n][0] << " " << result_log[n][1] << " " << result_log[n][2]  << "\n";
    }
    terminal << "\n\n\n";
}


/*
void forward_dynamics_identification()
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
 
    timer.delay_ms(200);

    led = 1;

    motor_control.set_velocity(0, 0);
    timer.delay_ms(200);
    
    
    uint32_t state      = 0;
    uint32_t time_next  = 0;  

    float wheel_diameter= 32.0;             //wheel diameter in mm
    float speed_max     = 50*2.0*PI/60.0;   //RPM to rad/s
    float distance_max  = 100.0;            //distance to travel in mm
    distance_max        = (2.0*PI)*distance_max/(wheel_diameter*PI);  //mm to radians

    float speed = speed_max; 

    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {   
        float distance      = (motor_control.get_left_position() + motor_control.get_right_position())/2.0;
        float velocity      = (motor_control.get_left_velocity() + motor_control.get_right_velocity())/2.0;

        if (state == 0 && distance > distance_max)
        { 
            speed  = 0;
            time_next = timer.get_time() + 250;
            state = 1;
        }  
        else if (state == 1 && timer.get_time() > time_next)
        {
            speed  = -speed_max;
            state = 2;
        }
        else if (state == 2 && distance < -distance_max)
        {
            speed  = 0;
            time_next = timer.get_time() + 250;
            state = 3;
        }
        else if (state == 3 && timer.get_time() > time_next)
        {
            speed  = speed_max;
            state = 0;
        }
    
        motor_control.set_velocity(speed, speed);
        timer.delay_ms(4); 

        result_log[n][0] = speed;
        result_log[n][1] = velocity;
        result_log[n][2] = distance; 
    }

    motor_control.set_velocity(0, 0); 
    timer.delay_ms(100);
    
    terminal << "\n\n\n"; 
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
        terminal << n << " " << result_log[n][0] << " " << result_log[n][1] << " " << result_log[n][2] << "\n";
    }
    terminal << "\n\n\n";
}
*/
