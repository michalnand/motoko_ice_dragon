#include "position_control_lqg.h"
#include <drivers.h>
#include <fmath.h>



PositionControlLQG *g_position_control_lqg;

#ifdef __cplusplus
extern "C" {
#endif  

/*
void TIM7_IRQHandler(void)
{ 
    g_position_control_lqg->callback();
    TIM_ClearITPendingBit(TIM7, TIM_IT_CC1);  
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

    //robot dimensions
    this->wheel_diameter = 34.0*0.001;
    this->wheel_brace    = 80.0*0.001;

    //1200RPM max, to rad/s
    float antiwindup = 1200*2.0*PI/60.0;


    //input shaper ramps 
    float du_p = 0.5;
    float du_n = -2.0;

    //init LQG (LQR with Kalman observer) 

    float mat_a[] = {
		1.0001942, 1.6989966e-06, 0.007875604, 0.0009928172, 
		-0.0005337515, 1.0004458, 2.2487402, -0.12869346, 
		0.00019399085, 1.7007355e-06, 0.008591391, 0.0009844207, 
		-0.0005337489, 0.0004457849, 2.2487316, -0.12869215 };

    float mat_b[] = {
            3.5510842e-05, 3.3929868e-05, 
            -0.0010557491, 0.0008889191, 
            3.5478788e-05, 3.3911936e-05, 
            -0.0010557477, 0.0008889183 };

    float mat_c[] = {
            1.0, 0.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 0.0 };

    float k[] = {
            910.45795, -29.55551, -49.44718, 4.032002, 
            921.3549, 29.50125, 66.909966, -2.4442132 };

    float ki[] = {
            28.143831, -0.72764903, 
            28.355661, 0.72218627 };

    float f[] = {
            0.6180795, 0.0009016679, 
            0.0009016679, 0.8721142, 
            5.5975008e-05, 0.0025088456, 
            0.0019244041, 0.64249945 };

        
    //controller init
    lqg.init(mat_a, mat_b, mat_c, k, ki, f, antiwindup);


    //shaper init
    shaper_left.init(du_p, du_n); 
    shaper_right.init(du_p, du_n); 

    //required values init
    this->x     = 0.0;
    this->theta = 0.0;

    /*
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
    */

    steps = 0;
    terminal << "position_control init [DONE]\n";
}

// @param : x : required distance, [m]
// @param : theta : required angle, [rad]
void PositionControlLQG::step(float x, float theta)
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