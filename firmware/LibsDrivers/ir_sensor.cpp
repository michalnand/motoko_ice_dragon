#include "ir_sensor.h"
#include "drivers.h"




/*
    IR left         :   PB1     ADC1, ADC2, IN9
    IR front left   :   PC5     ADC1, ADC2, IN15
    IR front right  :   PC4     ADC1, ADC2, IN14
    IR right        :   PB0     ADC1, ADC2, IN8
*/


//cubic polynomial calibration coefficients
const float ir_calibration[] = 
{
    -6.08119998e+01,  2.50778924e-01, -1.72371335e-04,  4.30063148e-08,
    -8.70491608e+01,  2.80947098e-01, -1.79185738e-04,  4.12438948e-08,
    -2.77370776e+02,  9.92379652e-01, -8.87244399e-04,  2.57949395e-07,
    -9.53491031e+01,  3.79776430e-01, -3.43710052e-04,  1.12921640e-07
}; 


void IRSensor::init()
{
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        //filters[i].init();
    }

    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        ir_off[i]   = 0;
        ir_on[i]    = 0;
        distance[i] = 0;
    }

   
    //initial state
    ir_led          = 0;
    state           = 0;

    filter_coeef    = 0.1;
}


void IRSensor::callback()
{
    if (state == 0)
    {
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_off[i] = adc.get()[i + IR_SENSOR_OFFSET];
        }

        //turn on IR led for next step
        ir_led = 1;
        state  = 1;
    }
    else
    {
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_on[i] = adc.get()[i + IR_SENSOR_OFFSET];
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
        float d     = calibration((float*)(ir_calibration + i*4), dif);
      
        //filter values
        distance[i] = (1.0 - filter_coeef)*distance[i] + filter_coeef*d;
    }
}
 

float* IRSensor::get()
{
    return distance;
}

float IRSensor::calibration(float *callibration, float x) 
{
    float y; 

    //cubic calibration
    y = callibration[0];
    y+= callibration[1]*x;
    y+= callibration[2]*x*x;
    y+= callibration[3]*x*x*x;
    
    return y;
}