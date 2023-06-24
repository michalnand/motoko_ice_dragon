#include <line_sensor.h>
#include <drivers.h>

LineSensor *g_line_sensor_ptr;

#ifdef __cplusplus
extern "C" {
#endif 


void TIM5_IRQHandler(void)
{ 
    g_line_sensor_ptr->callback();
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);  
} 
 
#ifdef __cplusplus
}
#endif

LineSensor::LineSensor()
{

}

LineSensor::~LineSensor()
{

}

void LineSensor::init(int dt)
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

    result.average = 0.0;

    /*
    //init timer 7, for dt time step
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
 
    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = (216*1000*dt);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM7, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM7, ENABLE); 

     
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    */


    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = (216*1000*dt);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM5, ENABLE); 

    
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
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
        adc_result[i] = 1000 - ((adc.get()[i] - adc_calibration_q[i])*1000)/adc_calibration_k[i];
        if (adc_result[i] < 0)
            adc_result[i] = 0;
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
