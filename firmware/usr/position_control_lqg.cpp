#include "position_control_lqg.h"
#include <drivers.h>
#include <fmath.h>



PositionControlLQG *g_position_control_lqg;

#ifdef __cplusplus
extern "C" {
#endif  

/*
void TIM5_IRQHandler(void)
{ 
    g_position_control_lqg->callback();
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);  
} 
*/

 
#ifdef __cplusplus
}
#endif


void PositionControlLQG::init()
{
    g_position_control_lqg = this;

    //TODO - move this into config.h

    //250Hz sampling rate, 4ms
    uint32_t dt_us = 4000;

    //robot dimensions, mm
    this->wheel_diameter = 34.0;
    this->wheel_brace    = 80.0;

    //1200RPM max, to rad/s
    float antiwindup = 1200*2.0*PI/60.0;


    //input shaper ramps 
    float du_p = 0.5;
    float du_n = -0.5;

    //init LQG (LQR with Kalman observer) 

    float mat_a[] = {
            1.0001942, 0.0, 0.007881245, 0.0, 
            0.0, 1.0004458, 0.0, -0.12871236, 
            0.00019422911, 0.0, 0.007881246, 0.0, 
            0.0, 0.00044578884, 0.0, -0.12871104 };

    float mat_b[] = {
            0.034720156, 0.034720156, 
            -0.00097235, 0.00097235, 
            0.034720156, 0.034720156, 
            -0.0009723489, 0.0009723489 };

    float mat_c[] = {
            1.0, 0.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 0.0 };

    float k[] = {
            1.4984868, -29.526764, 0.011306433, 3.2926738, 
            1.4984868, 29.526764, 0.011306433, -3.2926738 };

    float ki[] = {
            0.07432498, -0.7249198, 
            0.07432498, 0.7249198 };

    float f[] = {
            0.6180858, 0.0, 
            0.0, 0.6207114, 
            6.9791655e-05, 0.0, 
            0.0, 0.006191243 };


        
    //controller init
    lqg.init(mat_a, mat_b, mat_c, k, ki, f, antiwindup);


    //shaper init
    shaper_left.init(du_p, du_n); 
    shaper_right.init(du_p, du_n); 

    //required values init
    this->x     = 0.0;
    this->theta = 0.0;

    steps = 0;

    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(dt_us);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM5, ENABLE);  

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    terminal << "position_control init [DONE]\n";
}

// @param : x : required distance, [m]
// @param : theta : required angle, [rad]
void PositionControlLQG::set(float x, float theta)
{
    this->x     = x;
    this->theta = theta;
}
        
void PositionControlLQG::callback()
{
    //fill required values
    lqg.yr[0] = this->x;
    lqg.yr[1] = this->theta;

    //fill current state
    float right_position = motor_control.get_right_position();
    float left_position  = motor_control.get_left_position();

    float distance = 0.25*(right_position + left_position)*wheel_diameter;
    float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

    lqg.y[0]  = distance;
    lqg.y[1]  = angle;

    //compute controller output
    lqg.step();

    //output shaping
    float vl_shaped = shaper_left.step(lqg.u[0]); 
    float vr_shaped = shaper_right.step(lqg.u[1]);

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