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

    //dims in mm
    this->wheel_diameter = 34.0;
    this->wheel_brace    = 80.0;

    //max speed, 1500rpm
    this->speed_max     = 1500*2.0*PI/60.0;

    float dist_ramp  = 0.5; 
    float angle_ramp = 0.1;

    distance_shaper.init(dist_ramp);
    angle_shaper.init(angle_ramp);

    float mat_a[] = {
      0.9996948, -0.0024726202, 0.7173754, 4.1855664, 
      2.344187e-08, 1.0000275, 0.0015241175, 0.20310567, 
      -0.00030514644, -0.0024726202, 0.7173754, 4.1855664, 
      2.3442079e-08, 2.7533026e-05, 0.001524116, 0.2031063 };

  float mat_b[] = {
      4.5308847, 1.9240154, 
      -0.2135496, 0.18302794, 
      4.5308847, 1.9240154, 
      -0.21354944, 0.1830278 };

  float mat_c[] = {
      1.0, 0.0, 0.0, 0.0, 
      0.0, 1.0, 0.0, 0.0, 
      0.0, 0.0, 1.0, 0.0, 
      0.0, 0.0, 0.0, 1.0 };

  float k[] = {
      0.0111122355, -0.08381365, 0.02097094, 0.12868126, 
      0.009244162, 0.110913455, 0.018990804, 0.16063412 };

  float ki[] = {
      0.00086349266, -0.0019752574, 0.0, 0.0, 
      0.0006678071, 0.0025652659, 0.0, 0.0 };

  float f[] = {
      0.7764054, 0.006789762, 0.18654844, 0.007995918, 
      0.006789762, 0.6184826, 0.00909337, 0.00055541546, 
      0.18654844, 0.00909337, 0.7540578, 0.012323017, 
      0.007995918, 0.00055541546, 0.012323017, 0.5007928 };

   
    //controller init
    lqg.init(mat_a, mat_b, mat_c, k, ki, f, 1.0);
    
    
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

void PositionControlLQG::set(float req_distance, float req_angle)
{
    this->req_distance  = req_distance;
    this->req_angle     = req_angle;
}
        
void PositionControlLQG::callback()
{
    //fill required values
    lqg.yr[0] = this->distance_shaper.step(this->req_distance);    // position
    lqg.yr[1] = this->angle_shaper.step(this->req_angle); // angle
    lqg.yr[2] = 0;
    lqg.yr[3] = 0;

    //fill current state
    float left_position  = motor_control.get_left_position();
    float right_position = motor_control.get_right_position();

    float left_velocity  = motor_control.get_left_velocity();
    float right_velocity = motor_control.get_right_velocity();


    this->distance = 0.25*(right_position + left_position)*wheel_diameter;
    this->angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

    this->distance_velocity = 0.25*(right_velocity + left_velocity)*wheel_diameter;
    this->angle_velocity    = 0.5*(right_position - left_velocity)*wheel_diameter / wheel_brace;
   
 
    lqg.y[0]  = this->distance; 
    lqg.y[1]  = this->angle;
    lqg.y[2]  = this->distance_velocity;  
    lqg.y[3]  = this->angle_velocity;  
 
    
    //compute controller output 
    lqg.step();

    
    float v_left_req  = this->speed_max*lqg.u[0]; 
    float v_right_req = this->speed_max*lqg.u[1]; 
    

    // send to wheel velocity controll
    motor_control.set_velocity(v_left_req, v_right_req);


    steps++;
}

