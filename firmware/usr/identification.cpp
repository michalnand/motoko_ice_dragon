#include <identification.h>
#include <drivers.h>


#define LED_GPIO        TGPIOE
#define LED_PIN         2

#define SAMPLES_COUNT   ((uint32_t)1000)

void turn_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    float u_amplitude = 0.15;  //input amplitude
    float x_amplitude = 0.05;  //output amplitude
    
    //4ms sampling period
    uint32_t dt    = 4;


    float u = u_amplitude; 


    float result_log[SAMPLES_COUNT][3];
    
    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
       result_log[n][0] = 0;
       result_log[n][1] = 0;
       result_log[n][2] = 0; 
    }

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    timer.delay_ms(500);

    for (unsigned int n = 0; n < SAMPLES_COUNT; n++)
    {
        led = 1;
        motor_control.set_torque(-u*MOTOR_CONTROL_MAX, u*MOTOR_CONTROL_MAX);

        float angular_rate  = gyro.get_angular_rate();
        float angle         = gyro.get_angle();

        result_log[n][0] = u;
        result_log[n][1] = angular_rate;
        result_log[n][2] = angle;

        if (angle > x_amplitude)
        {
            u = -u_amplitude;
        }
        else if (angle < -x_amplitude)
        {
            u = u_amplitude;
        }
        led = 0;

        timer.delay_ms(dt);
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