#include "position_control.h"


//timer 7 interrupt handler
PositionControl *g_position_control_ptr;

#ifdef __cplusplus
extern "C" {
#endif

void TIM7_IRQHandler(void)
{ 
    g_position_control_ptr->callback();
    TIM_ClearITPendingBit(TIM7, TIM_IT_CC1);  
} 
 
#ifdef __cplusplus
}
#endif

 
void PositionControl::init()
{
    g_position_control_ptr = this;
    lqr.init(nullptr, nullptr, 1.0, POSITION_CONTROL_DT);

    line_following_mode = false;

    //init timer interrupt
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(POSITION_CONTROL_DT*1000); 
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

}

void PositionControl::set_required(float dtheta, float dx)
{
    lqr.xr[1] = lqr.x[1] + dtheta;
    lqr.xr[3] = lqr.x[3] + dx;
}

void PositionControl::set_state(uint32_t idx, float x)
{
    lqr.x[idx] = x;
}


void PositionControl::callback()
{
    //line following mode
    if (line_following_mode == true)
    {
        lqr.x[0] = line_sensor.result.angular_rate;
        lqr.x[1] = line_sensor.result.angle;
    }
    //generic robot position control
    else
    {
        lqr.x[0] = gyro.angular_rate_z;
        lqr.x[1] = gyro.angle_z;
    }

    //robot x position
    lqr.x[2] = motor_control.velocity;
    lqr.x[3] = motor_control.distance;

    lqr.step();

    int32_t left_torque  = lqr.u[0]*MOTOR_CONTROL_MAX;
    int32_t right_torque = lqr.u[1]*MOTOR_CONTROL_MAX;

    motor_control.set_torque(left_torque, right_torque);
}