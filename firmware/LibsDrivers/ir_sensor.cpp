#include "ir_sensor.h"
#include "drivers.h"
#include "fmath.h"






//cubic polynomial calibration coefficients


//robot A
const float ir_calibration[] = 
{
    -2.29540903e+01,  2.96361755e-01, -2.62612931e-04,  8.66668161e-08,
     9.06688653e+00,  6.64371251e-02, -4.04385662e-05,  9.68716433e-09,
    -1.52182976e+01,  9.69920014e-02, -5.52549401e-05,  1.16107679e-08,
    -3.66428832e+00,  1.97931509e-01, -1.71666039e-04,  6.12578482e-08
}; 


/*
//robot B
const float ir_calibration[] = 
{
    9.80249682e+00,  9.26234367e-02, -5.90240728e-05,  1.77460804e-08,
    1.43986983e+01,  6.88630342e-02, -3.69489076e-05,  8.81606417e-09,
    1.09433293e+01,  6.61345111e-02, -4.17044185e-05,  1.02359695e-08,
     1.83792407e+01,  4.68946392e-02, -2.65283530e-05,  8.01138781e-09
}; 
*/


void IRSensor::init()
{
    terminal << "ir_sensor init start\n";

    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        ir_off[i]   = 0;
        ir_on[i]    = 0;
        distance[i] = 0;
    }

   
    //initial state
    ir_led          = 0;
    state           = 0;

    filter_coeff    = 0.1;

    measurement_id  = 0;

    terminal << "ir_sensor init [DONE]\n";
}   


void IRSensor::callback()
{
    measurement_id++;

    if (state == 0) 
    {
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_off[i] = adc.get()[i + IR_SENSOR_OFFSET];
        }

        //turn on IR led for next step
        ir_led  = 1;
        state   = 1;
    }
    //wait state
    else if (state == 1)
    {
        state = 2;      
    }
    else if (state == 2)
    {
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_on[i] = adc.get()[i + IR_SENSOR_OFFSET];
        }
        
        //turn off IR led for next step
        ir_led  = 0;
        state   = 3;
    }   
    //wait state
    else     
    {
        state   = 0;  
    }
  

    //if dif is small obstacle is close
    //bigger value, bigger distance
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        //difference
        int dif = 4096 - (ir_off[i] - ir_on[i]);

        //compute distance from raw readings
        float d = calibration((float*)(ir_calibration + i*4), dif);
        
        //float d = dif;

        //filter values
        distance[i] = (1.0 - filter_coeff)*distance[i] + filter_coeff*d;
    }
}

float IRSensor::obstacle_distance()
{
    return min(distance[2], distance[3]);
}   

int IRSensor::obstacle_detected()
{
    float d = obstacle_distance();

    if (d < 120.0)         
    {
        return 2;
    }
    else if (d < 150.0)
    {
        return 1;
    }
    else
    {
        return 0;
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

    if (y < 0.0)
    {
        y = 0.0;
    }

    if (y > 200.0)
    {
        y = 200.0;
    }

    return y;
}


void IRSensor::print()
{
    terminal << "ir sensor\n";
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        terminal << ir_sensor.get()[i] << " ";
    }
    terminal << "\n\n\n";
}