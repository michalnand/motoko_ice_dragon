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
}

void PositionControl::set(float dtheta, float dx)
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
    lqr.step();

    int32_t left_torque  = lqr.u[0]*MOTOR_CONTROL_MAX;
    int32_t right_torque = lqr.u[1]*MOTOR_CONTROL_MAX;

    motor_control.set_torque(left_torque, right_torque);
}