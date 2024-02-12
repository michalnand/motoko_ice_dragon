#include "position_control.h"
#include <drivers.h>



PositionControl *g_position_control;

#ifdef __cplusplus
extern "C" {
#endif


void TIM7_IRQHandler(void)
{ 
    g_position_control->callback_position();
    TIM_ClearITPendingBit(TIM7, TIM_IT_CC1);  
} 

 
#ifdef __cplusplus
}
#endif




void PositionControl::init(float du_p_max, float du_n_max, uint32_t dt_us)
{
    g_position_control = this;

    this->du_p_max     = du_p_max/(dt_us*0.000001);
    this->du_n_max     = du_n_max/(dt_us*0.000001);
    this->u_left_curr  = 0.0;
    this->u_right_curr = 0.0;

    this->u_left_saturation  = false;
    this->u_right_saturation = false;

    this->x     = 0.0;
    this->theta = 0.0;



    //init timer 7 interrupt for callback calling, 250Hz
    
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(dt_us);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM7, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM7, ENABLE);  

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    
    terminal << "position_control init [DONE]\n";
}


void PositionControl::step(float x, float theta)
{
    this->x = x;
    this->theta = theta;
}
        
void PositionControl::callback_position()
{
    //TODO : LQR here
    float u_left  = 123;
    float u_right = 456;

    float dif;

    // trapezoidal left wheel control change limitation     
    dif = u_left - u_left_curr;

    if (dif > du_p_max)
    {
        u_left_curr+= du_p_max;
        u_left_saturation = true;
    }
    else if (dif < du_n_max)
    {
        u_left_curr+= du_n_max;
        u_left_saturation = true;
    }
    else
    {
        u_left_curr = u_left;
        u_left_saturation = false;
    }


    // trapezoidal right wheel control change limitation     
    dif = u_right - u_right_curr;

    if (dif > du_p_max)
    {
        u_right_curr+= du_p_max;
        u_right_saturation = true;
    }
    else if (dif < du_n_max)
    {
        u_right_curr+= du_n_max;
        u_right_saturation = true;
    }
    else
    {
        u_right_curr = u_right;
        u_right_saturation = false;
    } 

    // send to wheel velocity controll
    motor_control.set_velocity(u_left_curr, u_right_curr);
}

