#include <identification.h>
#include <drivers.h>

#define LED_GPIO        TGPIOE
#define LED_PIN         2



void turn_dynamics_identification()
{
    float u_amplitude = 0.2*MOTOR_CONTROL_MAX;  //input amplitude
    float x_amplitude = 20.0;                   //output amplitude
    
    //4ms sampling period
    uint32_t dt    = 4;


    uint32_t state = 0;
    float u = 0.0; 

    while (1)
    {
        float x = line_sensor.result.center_line_position;

        if (state == 0)
        {
            //start turning into one side
            u = u_amplitude;
            if (x > x_amplitude)
            {
                //switch turning side
                state = 1;
            }
        }
        else
        {
            //oposite side
            u = -u_amplitude;
            if (x < -x_amplitude)
            {
                //switch turning side back
                state = 0;
            }
        }

        motor_control.set_torque(u, -u); 
        timer.delay_ms(dt);
    }
}