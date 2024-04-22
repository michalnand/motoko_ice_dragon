#include <line_sensor.h>
#include <drivers.h>
#include <fmath.h>

LineSensor *g_line_sensor_ptr;

LineSensor::LineSensor()
{

}

LineSensor::~LineSensor()
{

}

void LineSensor::init()
{
    terminal << "line_sensor init start\n";

    g_line_sensor_ptr = this;

    sensor_led = 0;
    timer.delay_ms(100);

    for (unsigned int i = 0; i < adc_result.size(); i++)
        adc_calibration_q[i] = adc.get()[i];
    
    sensor_led = 1;
    timer.delay_ms(100); 

    for (unsigned int i = 0; i < adc_calibration_k.size(); i++)
        adc_calibration_k[i] =  adc.get()[i] - adc_calibration_q[i];

    for (unsigned int i = 0; i < adc_result.size(); i++)
        adc_result[i] = 0;


    weights[0] =  4*LINE_SENSOR_STEP;
    weights[1] =  3*LINE_SENSOR_STEP; 
    weights[2] =  2*LINE_SENSOR_STEP;
    weights[3] =  1*LINE_SENSOR_STEP;
    weights[4] = -1*LINE_SENSOR_STEP;
    weights[5] = -2*LINE_SENSOR_STEP;
    weights[6] = -3*LINE_SENSOR_STEP;
    weights[7] = -4*LINE_SENSOR_STEP;


    line_lost_type = LINE_LOST_CENTER;
    on_line_count  = 0;

    left_position = 0.0;
    right_position = 0.0;

    minimal_position = 0.0;
    extremal_position = 0.0; 

    left_angle = 0.0;
    right_angle = 0.0;

    measurement_id = 0;

    terminal << "line_sensor init [DONE]\n";
}


void LineSensor::callback()  
{
    uint32_t state = adc.measurement_id%ADC_CHANNELS_COUNT;

    if (state == 0)
    {
        for (unsigned int i = 0; i < adc_result.size(); i++)
        { 
            int v = 1000 - ((adc.get()[i] - adc_calibration_q[i])*1000)/adc_calibration_k[i];

            if (v < 0)
            {
                v = 0;
            }       

            //low pass filter
            adc_result[i] = (3*adc_result[i] + v)/4; 
        }   
       
        process();
    }
}

void LineSensor::print()
{
    terminal << "line sensor\n";

    terminal << "adc_result : ";
    for (unsigned int i = 0; i < adc_result.size(); i++)
        terminal << adc_result[i] << " ";
    terminal << "\n";

    for (unsigned int i = 0; i < adc_calibration_q.size(); i++)
        terminal << adc_calibration_q[i] << " ";
    terminal << "\n";

    for (unsigned int i = 0; i < adc_calibration_k.size(); i++)
        terminal << adc_calibration_k[i] << " ";
    terminal << "\n";

    terminal << "\n\n";


    terminal << "\n";   

    terminal << "line_lost_type =   " << line_lost_type << "\n";
    terminal << "on_line_count  =   " << on_line_count << "\n";
    terminal << "left_position  =   " << left_position << "\n";
    terminal << "right_position =   " << right_position << "\n";
    terminal << "left_angle     =   " << (float)(left_angle*180.0/PI) << " deg \n";
    terminal << "right_angle    =   " << (float)(right_angle*180.0/PI) << " deg \n";
    
    terminal << "\n\n\n";
}


void LineSensor::process()
{    
    //compute average of all sensors
    int average = 0;
    for (unsigned int i = 0; i < adc_result.size(); i++)
        average+= adc_result[i];
    average = average/adc_result.size();
 
 
    //find most left sensor on line
    unsigned int left_idx = 0;
    bool left_valid = false;
    for (int i = (adc_result.size()-1); i >= 0; i--)
        if (adc_result[i] > LINE_SENSOR_THRESHOLD)
        {
            left_idx = i;
            left_valid = true; 
            break;
        }

    //find most right sensor on line 
    unsigned int right_idx = 0;
    bool right_valid = false;
    for (int i = 0; i < (int)adc_result.size(); i++)
        if (adc_result[i] > LINE_SENSOR_THRESHOLD)
        {
            right_idx = i;
            right_valid = true;
            break;
        }


    //compute line position arround strongest sensors
    float k = 1.0/((LINE_SENSOR_COUNT/2)*LINE_SENSOR_STEP);

   
    if (left_valid)
    {
        left_position  = k*integrate(left_idx);
        right_position = left_position;

    }

    if (right_valid)
    {
        right_position  = k*integrate(right_idx);
    }   

  
    //solve if line lost
    if ((left_valid == false) && (right_valid == false))
    {
        if (left_position < -0.8)
            line_lost_type = LINE_LOST_LEFT;
        else if (left_position > 0.8) 
            line_lost_type = LINE_LOST_RIGHT;
        else
            line_lost_type = LINE_LOST_CENTER;
    }
    else
    {
        line_lost_type  = LINE_LOST_NONE;
    }
    
    if (abs(left_position) < abs(right_position))
    { 
        minimal_position = left_position;
        extremal_position= right_position;
    }
    else
    {
        minimal_position = right_position;
        extremal_position= left_position;
    }
    

    //compute to robot angle in radians
    left_angle  = fatan(left_position * (SENSORS_BRACE/2.0) / SENSORS_DISTANCE);
    right_angle = fatan(right_position * (SENSORS_BRACE/2.0) / SENSORS_DISTANCE);

 
    measurement_id++;
}


int LineSensor::integrate(int center_idx)
{
    if (center_idx < 0)
        center_idx = 0;

    if (center_idx > (int)(LINE_SENSOR_COUNT-1))
        center_idx = (int)(LINE_SENSOR_COUNT-1);

    int center  = weights[center_idx]*adc_result[center_idx];
    int sum     = adc_result[center_idx];

    int int_result = center;

    if (center_idx > 0)
    {
        int_result+= weights[center_idx - 1]*adc_result[center_idx - 1];
        sum+= adc_result[center_idx - 1];
    }
    else
    {
        int_result+= center;
        sum+= adc_result[center_idx];
    }

    if (center_idx < (int)(LINE_SENSOR_COUNT-1))
    {
        int_result+= weights[center_idx+1]*adc_result[center_idx+1];
        sum+= adc_result[center_idx+1];
    }
    else
    {
        int_result+= center;
        sum+= adc_result[center_idx];
    }

    int_result = int_result/sum;

    return int_result;
}
