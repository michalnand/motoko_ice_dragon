#include "position_control_lqr.h"
#include <drivers.h>
#include <fmath.h>


PositionControlLQR position_control;

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

    float dist_ramp  = 5.0; 
    float angle_ramp = 10.0;  
 
    distance_shaper.init(dist_ramp, 0.98);
    angle_shaper.init(angle_ramp, 0.2);

    /*
    // q = 10**-7, 10**-3
    float k[] = {
      0.0052479296, -0.33005857, 0.011264713, 0.08377834, 
      0.0052479296, 0.33005857, 0.011264713, 0.08377834 };

    float ki[] = {
      0.00023484259, -0.024085023, 0.0, 0.0, 
      0.00023484259, 0.024085023, 0.0, 0.0 };
    */

    // q = 10**-7, 0.5*10**-4 
    float k[] = {
      0.005248046, -0.27352497, 0.011264535, 0.08430575, 
      0.005248046, 0.27352497, 0.011264535, 0.08430575 };

    float ki[] = {
		  0.00023489761, -0.016833698, 0.0, 0.0, 
		  0.00023489761, 0.016833698, 0.0, 0.0 };
    
    /*
    // q = 10**-7, 10**-4
    float k[] = {
      0.0052446346, -0.17818446, 0.011256655, -0.084595665, 
      0.0052446346, 0.17818446, 0.011256655,   0.084595665 };

    float ki[] = {
      0.000234812, -0.007370741, 0.0, 0.0, 
      0.000234812, 0.007370741, 0.0, 0.0 };
    */

    //controller init
    lqr.init(k, ki, 1.0);

    
    //required values init
    this->req_distance  = 0.0;
    this->req_angle     = 0.0;

    this->distance_prev  = 0.0;
    this->distance       = 0.0;
    this->angle_prev     = 0.0;
    this->angle          = 0.0;

    this->line_angle_prev = 0.0;
    this->line_angle      = 0.0;

    this->lf_mode = false;
    this->steps = 0;
   
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
 
void PositionControlLQR::set_circle_motion(float radius, float speed)
{
    //obtain current state  
    float distance = this->distance;  
    float angle    = this->angle;

    //calculate motion change

    float vc = speed;  
    float va = speed/radius;

    this->req_distance = distance + vc;
    this->req_angle    = angle    + va;
}


bool PositionControlLQR::on_target(float distance_threshold, float angle_threshold)
{
  if (abs(this->req_distance - this->distance) > distance_threshold)
  {
    return false;
  }

  if (abs(this->req_angle - this->angle) > angle_threshold)
  {
    return false;
  }

  return true;
}

void PositionControlLQR::enable_lf()
{
  if (this->lf_mode == false)
  {
    this->lf_mode = true;

    uint32_t steps_old = this->steps;
    
    while (steps_old == this->steps)
    {
      __asm("nop");
    }

    this->angle_prev = this->angle;
  }
}

void PositionControlLQR::disable_lf()
{
  if (this->lf_mode == true)
  {
    this->lf_mode = false;

    uint32_t steps_old = this->steps;
    
    while (steps_old == this->steps)
    {
      __asm("nop");
    }

    this->angle_prev = this->angle;
  }
}
        
void PositionControlLQR::callback()
{
    //fill required values
    lqr.xr[0] = this->distance_shaper.step(this->req_distance);
    //lqr.xr[1] = this->angle_shaper.step(this->req_angle);
    lqr.xr[1] = this->req_angle; 
    
    //fill current state 
    float left_position  = motor_control.get_left_position();
    float right_position = motor_control.get_right_position();

    this->distance_prev = this->distance;
    this->distance = 0.25*(right_position + left_position)*wheel_diameter;

    this->angle_prev = this->angle;
    this->angle      = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

    this->line_angle_prev = this->line_angle;
    this->line_angle  = line_sensor.left_angle;

    //in line following mode, read angular rate from line sensor
    if (this->lf_mode == true)
    {
      float angular_rate = this->line_angle  - this->line_angle_prev;

      lqr.x[0]  = this->distance; 
      lqr.x[1]  = this->angle;
      lqr.x[2]  = this->distance - this->distance_prev;  
      lqr.x[3]  = angular_rate; 
    } 
    else
    {
      lqr.x[0]  = this->distance; 
      lqr.x[1]  = this->angle;
      lqr.x[2]  = this->distance - this->distance_prev;  
      lqr.x[3]  = this->angle    - this->angle_prev;  
    }
    
    
    //compute controller output 
    lqr.step();

    // scale to motors
    float v_left_req  = this->speed_max*lqr.u[0]; 
    float v_right_req = this->speed_max*lqr.u[1]; 

    // send to wheel velocity controll
    motor_control.set_velocity(v_left_req, v_right_req);

    steps++;
}