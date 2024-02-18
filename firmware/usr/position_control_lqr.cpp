#include "position_control_lqr.h"
#include <drivers.h>
#include <fmath.h>



PositionControlLQR *g_position_control_lqr;

#ifdef __cplusplus
extern "C" {
#endif 


void TIM7_IRQHandler(void)
{ 
    g_position_control_lqr->callback();
    TIM_ClearITPendingBit(TIM7, TIM_IT_CC1);  
} 
 
 
#ifdef __cplusplus
}
#endif


void PositionControlLQR::init()
{
    g_position_control_lqr = this;

    //TODO - move this into config.h

    //250Hz sampling rate, 4ms
    uint32_t dt_us = 4000;

    //robot dimensions
    this->wheel_diameter = 34.0*0.001;
    this->wheel_brace    = 80.0*0.001;

    //1200RPM max, to rad/s
    float antiwindup = 1200*2.0*PI/60.0;


    //input shaper ramps
    float du_p = 0.5;
    float du_n = -2.0;

    //init LQR 

    float k[] = {
		687.7318, -29.65278, 
		694.7266, 29.526167 };

    float ki[] = {
		16.122814, -0.7278881, 
		16.255505, 0.7219375 };
        
    //controller init
    lqr.init(k, ki, antiwindup);

    //shaper init
    shaper_left.init(du_p, du_n); 
    shaper_right.init(du_p, du_n); 

    //required values init
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
    


    steps = 0;
    terminal << "position_control init [DONE]\n";
}

// @param : x : required distance, [m]
// @param : theta : required angle, [rad]
void PositionControlLQR::step(float x, float theta)
{
    this->x     = x;
    this->theta = theta;
}
        
void PositionControlLQR::callback()
{
    //fill required values
    lqr.xr[0] = this->x;
    lqr.xr[1] = this->theta;

    //fill current state
    float right_position = motor_control.get_right_position();
    float left_position  = motor_control.get_left_position();

    float distance = 0.25*(right_position + left_position)*wheel_diameter;
    float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

    lqr.x[0]  = distance;
    lqr.x[1]  = angle;

    //compute controller output
    lqr.step();

    //output shaping
    float vl_shaped = shaper_left.step(lqr.u[0]); 
    float vr_shaped = shaper_right.step(lqr.u[1]);

    // send to wheel velocity controll
    motor_control.set_velocity(vl_shaped, vr_shaped);
    
    
    /*
        
    if ((steps%50) == 0)
    {
        terminal << lqr.integral_action[0] << " " << lqr.integral_action[1] << " " << lqr.u[0] << " " << lqr.u[1] << "\n";
    }

    steps++;
    */
}