#include "ir_sensor.h"


void IRSensor::init()
{
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        filters[i].init();
    }

    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        ir_off[i]   = 0;
        ir_on[i]    = 0;
        distance[i] = 0;
    }

    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        cal_a[i] = 0.0;
        cal_b[i] = 1.0;
        cal_c[i] = 0.0;
    }
     
    //initial state
    state  = 0;
    ir_led = 0;

    //TODO init timer + ADC interrupt for callback calling
}


void IRSensor::callback()
{
    if (state == 0)
    {
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_off[i] = 123; //TODO  ADC read
        }

        //turn on IR led for next step
        ir_led = 1;
        state  = 1;
    }
    else
    {
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_on[i] = 123; //TODO  ADC read
        }

        //turn off IR led for next step
        ir_led = 0;
        state  = 0;
    }

  
    //if dif is small obstacle is close
    //bigger value, bigger distance
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        //difference
        int dif     = 4096 - (ir_off[i] - ir_on[i]);

        //compute distance from raw readings
        float d     = calibration(i, dif);

        //filter values
        distance[i] = filters[i].step(d, IR_SENSORS_VARIANCE);
    }
}


float IRSensor::calibration(int idx, float x) 
{
    float y = 0.0; 
    
    //quadratic calibration
    y+= cal_a[idx]*x*x;
    y+= cal_b[idx]*x; 
    y+= cal_c[idx];
    
    return y;
}