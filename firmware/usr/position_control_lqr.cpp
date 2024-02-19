#include "position_control_lqr.h"
#include <drivers.h>
#include <fmath.h>



PositionControlLQR *g_position_control_lqr;

#ifdef __cplusplus
extern "C" {
#endif 


void TIM5_IRQHandler(void)
{ 
    g_position_control_lqr->callback();
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);  
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

    //dims in mm
    this->wheel_diameter = 34.0;
    this->wheel_brace    = 80.0;

    //max speed, 1500rpm
    this->speed_max     = 1500*2.0*PI/60.0;

    float dist_ramp  = 10.0; 
    float angle_ramp = 180.0*PI/180.0;

    distance_shaper.init(dist_ramp, -dist_ramp);
    angle_shaper.init(angle_ramp, -angle_ramp);


    float k[] = {
		  0.01783013, -0.1605672, 
		  0.015368554, 0.19272842 };

    float ki[] = {
      0.0028430824, -0.0067023537, 
      0.0023793622, 0.00798543 };


   
    //controller init
    lqr.init(k, ki, 1.0);

    
    //required values init
    this->req_distance  = 0.0;
    this->req_angle     = 0.0;

    this->distance = 0.0;
    this->angle    = 0.0;

    this->distance_velocity = 0.0;
    this->angle_velocity    = 0.0;

    steps = 0;
   
    //init timer 5 interrupt for callback calling, 250Hz
    
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

void PositionControlLQR::set(float req_distance, float req_angle)
{
    this->req_distance  = req_distance;
    this->req_angle     = req_angle;
}
        
void PositionControlLQR::callback()
{
    //fill required values
    lqr.xr[0] = this->distance_shaper.step(this->req_distance);    // position
    lqr.xr[1] = this->angle_shaper.step(this->req_angle); // angle

    //fill current state
    float left_position  = motor_control.get_left_position();
    float right_position = motor_control.get_right_position();

    float left_velocity  = motor_control.get_left_velocity();
    float right_velocity = motor_control.get_right_velocity();


    this->distance = 0.25*(right_position + left_position)*wheel_diameter;
    this->angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

    this->distance_velocity = 0.25*(right_velocity + left_velocity)*wheel_diameter;
    this->angle_velocity    = 0.5*(right_position - left_velocity)*wheel_diameter / wheel_brace;
   
 
    lqr.x[0]  = this->distance; 
    lqr.x[1]  = this->angle;
    //lqr.x[2]  = this->distance_velocity;  
    //lqr.x[3]  = this->angle_velocity;  
 
    
    //compute controller output 
    lqr.step();

    
    float v_left_req  = this->speed_max*lqr.u[0]; 
    float v_right_req = this->speed_max*lqr.u[1]; 
    

    // send to wheel velocity controll
    motor_control.set_velocity(v_left_req, v_right_req);


    steps++;
}

