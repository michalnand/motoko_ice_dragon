#include <drivers.h>

#include <motor_control.h>
#include <gonio.h>
 
//helping stuff
#define SQRT3       ((int32_t)1773)      // sqrt(3)     = 1773/1024
#define SQRT3INV    ((int32_t)591)       // 1/sqrt(3)   = 591/1024

#define max2(x,y) (((x) >= (y)) ? (x) : (y))
#define min2(x,y) (((x) <= (y)) ? (x) : (y))

#define max3(x, y, z) (max2(max2(x, y), z))
#define min3(x, y, z) (min2(min2(x, y), z))


 
//timer 2 interrupt handler, running commutation process, 4kHz
MotorControl *g_motor_control_ptr;

#ifdef __cplusplus
extern "C" {
#endif

void TIM2_IRQHandler(void)
{ 
    g_motor_control_ptr->callback();
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);  
} 
 
#ifdef __cplusplus
}
#endif



void MotorControl::init()
{
    terminal << "motor_controll init start\n";

    g_motor_control_ptr = this;

    left_pwm.init();
    right_pwm.init();

    hold();


    left_encoder.init();
    right_encoder.init();

    //init timer 2 interrupt for callback calling, 2kHz
    
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(MOTOR_CONTROL_DT); //(SystemCoreClock/2)*500;
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);  

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    set_torque(0, 0);

    terminal << "motor_controll init [DONE]\n";
}

void MotorControl::set_torque(int32_t left_torque, int32_t right_torque)
{
    this->left_torque  = clamp(left_torque,  -MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX);
    this->right_torque = clamp(right_torque, -MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX);
}
 

void MotorControl::callback()
{
    left_encoder.update(MOTOR_CONTROL_DT);  
    right_encoder.update(MOTOR_CONTROL_DT);   

    int32_t left_u;
    int32_t left_phase; 
    
    if (this->left_torque < 0) 
    {
        left_u     = -this->left_torque;
        left_phase = -SINE_TABLE_SIZE/4; 
        
    }
    else
    {
        left_u     = this->left_torque;
        left_phase = SINE_TABLE_SIZE/4; 
    }

    int32_t right_u;
    int32_t right_phase; 

    if (this->right_torque < 0) 
    {
        right_u     = -this->right_torque;
        right_phase = SINE_TABLE_SIZE/4; 
        
    }
    else
    {
        right_u     = this->right_torque;
        right_phase = -SINE_TABLE_SIZE/4; 
    }

    set_torque_from_rotation(left_u,   left_phase,  left_encoder.angle,   0);
    set_torque_from_rotation(right_u,  right_phase, right_encoder.angle,  1);
}


void MotorControl::hold()
{
    set_torque_from_rotation(500,  0,   0,  0);
    set_torque_from_rotation(500,  0,   0,  1);
 
    timer.delay_ms(200);
}

int32_t MotorControl::get_left_angle()
{
    return left_encoder.angle;
}
     
float MotorControl::get_left_position()
{
    return -left_encoder.position/4096.0;
}

float MotorControl::get_left_velocity()
{
    return -left_encoder.angular_velocity/4096.0;
}




int32_t MotorControl::get_right_angle()
{
    return right_encoder.angle;
}

float MotorControl::get_right_position()
{
    return right_encoder.position/4096.0;
} 

float MotorControl::get_right_velocity()
{
    return right_encoder.angular_velocity/4096.0;
}



void MotorControl::set_torque_from_rotation(int32_t torque, int32_t phase, uint32_t rotor_angle, int motor_id)
{
    //field vector domain
    int32_t q = (torque*cos_tab(phase))/SINE_TABLE_MAX;
    int32_t d = (torque*sin_tab(phase))/SINE_TABLE_MAX; 

    uint32_t theta = (rotor_angle*MOTOR_POLES)/(2*4);


    //inverse Park transform
    int32_t alpha = (d*cos_tab(theta) - q*sin_tab(theta))/SINE_TABLE_MAX;
    int32_t beta  = (d*sin_tab(theta) + q*cos_tab(theta))/SINE_TABLE_MAX;


    //inverse Clarke transform
    int32_t a = alpha;
    int32_t b = -(alpha/2) + (SQRT3*beta)/(2*1024);
    int32_t c = -(alpha/2) - (SQRT3*beta)/(2*1024);


    //transform into space-vector modulation, to achieve full voltage range
    int32_t min_val = min3(a, b, c);
    int32_t max_val = max3(a, b, c); 

    int32_t com_val = (min_val + max_val)/2;  

    //normalise into 0..MOTOR_CONTROL_MAX
    int32_t a_pwm = ((a - com_val)*SQRT3INV)/1024 + MOTOR_CONTROL_MAX/2;
    int32_t b_pwm = ((b - com_val)*SQRT3INV)/1024 + MOTOR_CONTROL_MAX/2;
    int32_t c_pwm = ((c - com_val)*SQRT3INV)/1024 + MOTOR_CONTROL_MAX/2;
    
     
    a_pwm = clamp((a_pwm*PWM_PERIOD)/MOTOR_CONTROL_MAX, 0, PWM_PERIOD);
    b_pwm = clamp((b_pwm*PWM_PERIOD)/MOTOR_CONTROL_MAX, 0, PWM_PERIOD);
    c_pwm = clamp((c_pwm*PWM_PERIOD)/MOTOR_CONTROL_MAX, 0, PWM_PERIOD);


    if (motor_id == 0)
    {
        left_pwm.set(b_pwm, a_pwm, c_pwm);
    }
    else
    {
        right_pwm.set(a_pwm, b_pwm, c_pwm);
    }
}



int32_t MotorControl::clamp(int32_t value, int32_t min, int32_t max)
{
    if (value > max)
    {
        value = max;
    }

    if (value < min)
    {
        value = min;
    }

    return value;
}