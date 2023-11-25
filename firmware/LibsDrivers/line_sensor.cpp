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


    weights[0] = -4*LINE_SENSOR_STEP;
    weights[1] = -3*LINE_SENSOR_STEP; 
    weights[2] = -2*LINE_SENSOR_STEP;
    weights[3] = -1*LINE_SENSOR_STEP;
    weights[4] =  1*LINE_SENSOR_STEP;
    weights[5] =  2*LINE_SENSOR_STEP;
    weights[6] =  3*LINE_SENSOR_STEP;
    weights[7] =  4*LINE_SENSOR_STEP;


    time_prev= timer.get_time();
    time_now = time_prev;

    measurement_id = 0;
    line_lost_type = LINE_LOST_NONE;
    on_line_count  = 0;
    line_position  = 0.0;
    angle          = 0.0;
    angular_rate   = 0.0;

    for (unsigned int i = 0; i < 4; i++)
    {
        angle_prev[i] = 0.0;
    }

    terminal << "line_sensor init [DONE]\n";
}


void LineSensor::callback()
{
    uint32_t state = adc.measurement_id%8;

    if (state == 0)
    {
        for (unsigned int i = 0; i < adc_result.size(); i++)
        { 
            int v = 1000 - ((adc.get()[i] - adc_calibration_q[i])*1000)/adc_calibration_k[i];
            if (v < 0)
            {
                v = 0;
            }

            adc_result[i] = v;
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


    terminal << "\n";

    terminal << "measurement_id =   " << measurement_id << "\n";
    terminal << "line_lost_type =   " << line_lost_type << "\n";
    terminal << "on_line_count  =   " << on_line_count << "\n";
    terminal << "line_position  =   " << line_position << "\n";
    terminal << "angle          =   " << angle << "\n";
    terminal << "angular_rate   =   " << angular_rate << "\n";
  
    terminal << "\n\n\n";
}


void LineSensor::process()
{    
    //compute average of all sensors
    int average = 0;
    for (unsigned int i = 0; i < adc_result.size(); i++)
        average+= adc_result[i];
    average = average/adc_result.size();
 
 
    //find maximum sensor value
    unsigned int center_line_idx = 0;
    for (unsigned int i = 0; i < adc_result.size(); i++)
        if (adc_result[i] > adc_result[center_line_idx])
        {
            center_line_idx = i;
        }


    //update line position only if machine still on line
    if (adc_result[center_line_idx] > LINE_SENSOR_THRESHOLD)
    {
        //compute line position arround strongest sensors
        float k = 1.0/((LINE_SENSOR_COUNT/2)*LINE_SENSOR_STEP);

        //parabolic integration, raw line position from -1, to 1
        line_position = k*integrate(center_line_idx);

        //compute to robot angle
        angle           = fatan(line_position * (SENSORS_BRACE/2.0) / SENSORS_DISTANCE)/PI;

        //from past angles compute angular rate
        time_prev   = time_now;
        time_now    = timer.get_time();

        //1st order 1st difference
        float dt    = (float)(time_now - time_prev)/1000.0;
        float tmp   = first_difference_1(angle, angle_prev, dt);
        

        //low pass filter for angular rate, smooth the value
        angular_rate    = (7.0*angular_rate + 1.0*tmp)/8.0;

        line_lost_type  = LINE_LOST_NONE;
    }
    else
    //line lost, fill value of lost type
    {
        if (line_position < -0.75)
            line_lost_type = LINE_LOST_RIGHT;
        else
        if (line_position > 0.75)
            line_lost_type = LINE_LOST_LEFT;
        else
            line_lost_type = LINE_LOST_CENTER;

        /*
        //clear angular rate, to avoid robot kick when returns back on line
        for (unsigned int i = 0; i < 4; i++)
        {
            angle_prev[i] = 0.0;
        }
        */
    }

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
