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
    g_line_sensor_ptr = this;

    off();
    timer.delay_ms(100);

    for (unsigned int i = 0; i < adc_result.size(); i++)
        adc_calibration_q[i] = adc.get()[i];
    
    on(); 
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

    result.line_lost_type   = LINE_LOST_NONE;
    result.line_type        = LINE_TYPE_SINGLE;
    result.on_line_count    = 0;
    result.measurement_id   = 0;

    result.center_line_position = 0.0;
    result.left_line_position   = result.center_line_position;
    result.right_line_position  = result.center_line_position;

    time_prev= timer.get_time();
    time_now = time_prev;

    result.angle_prev   = 0;
    result.angle        = 0;
    result.angular_rate = 0;
        

    result.average = 0.0;



}


void LineSensor::on()
{
    sensor_led = 1;
}

void LineSensor::off()
{
    sensor_led = 0;
}

void LineSensor::callback()
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
    
    line_filter();
}

void LineSensor::print()
{
    terminal << "line sensor\n";

    terminal << "adc_result : ";
    for (unsigned int i = 0; i < adc_result.size(); i++)
        terminal << adc_result[i] << " ";
    terminal << "\n";


    terminal << "\n";

    terminal << "measurement_id = " << result.measurement_id << "\n";
    terminal << "lost type = " << result.line_lost_type << "\n";
    terminal << "type    = " << result.line_type << "\n";
    terminal << "count   = " << result.on_line_count << "\n";
    terminal << "average = " << result.average << "\n";
    terminal << "center  = " << result.center_line_position << "\n";
    terminal << "left    = " << result.left_line_position << "\n";
    terminal << "right   = " << result.right_line_position << "\n";

    terminal << "\n\n\n";
}


void LineSensor::line_filter()
{
    result.measurement_id++;
    result.line_lost_type = LINE_LOST_CENTER;
    
    //compute average of all sensors
    int average = 0;
    for (unsigned int i = 0; i < adc_result.size(); i++)
        average+= adc_result[i];
    average = average/adc_result.size();

    result.average = average;

    //find maximum sensor value
    unsigned int center_line_idx = 0;
    for (unsigned int i = 0; i < adc_result.size(); i++)
        if (adc_result[i] > adc_result[center_line_idx])
        {
            center_line_idx = i;
        }

    unsigned int on_line_count = 0;
    for (unsigned int i = 0; i < adc_result.size(); i++)
        if (adc_result[i] > LINE_SENSOR_THRESHOLD)
            on_line_count++;

    //compute line position
    float k = 1.0/((LINE_SENSOR_COUNT/2)*LINE_SENSOR_STEP);
    result.center_line_position = k*integrate(center_line_idx);

    float sensors_brace    = 69.5;
    float sensors_distance = 68.88;

    time_prev= time_now;
    time_now = timer.get_time();

    result.angle_prev   = result.angle;
    result.angle        = fatan(result.center_line_position*(sensors_brace/2.0) / sensors_distance)/PI;

    float tmp           = ((result.angle - result.angle_prev)*1000)/(time_now - time_prev);
    result.angular_rate = (7*result.angular_rate + 1*tmp)/8;
        
    /*
    if (average > 400)
    {
        result.line_type        = LINE_TYPE_SPOT;
        result.on_line_count    = on_line_count;
        result.line_lost_type   = LINE_LOST_NONE;
    }
    else
    if (adc_result[center_line_idx] > LINE_SENSOR_THRESHOLD)
    {
        float k = 1.0/((LINE_SENSOR_COUNT/2)*LINE_SENSOR_STEP);
        result.line_type = LINE_TYPE_SINGLE;
        result.center_line_position = k*integrate(center_line_idx);
        result.left_line_position   = k*integrate(center_line_idx + 1);
        result.right_line_position  = k*integrate(center_line_idx - 1);
        result.on_line_count = on_line_count;
        result.line_lost_type = LINE_LOST_NONE;
    }
    else
    {
        if (result.center_line_position < -0.75)
            result.line_lost_type = LINE_LOST_RIGHT;
        else
        if (result.center_line_position > 0.75)
            result.line_lost_type = LINE_LOST_LEFT;
        else
            result.line_lost_type = LINE_LOST_CENTER;
    }
    */
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
