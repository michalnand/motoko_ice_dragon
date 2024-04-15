#include "pwm.h"
#include <drivers.h>

void PWMLeft::init()
{
    //left motor PWM controll pins 
    //PWMA      : PC9, TIM3_CH4
    //PWMB      : PC8, TIM3_CH3
    //PWMC      : PC7, TIM3_CH2
    //enable    : PA8 

    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 

    //init pins
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_9 | GPIO_Pin_8 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Medium_Speed;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    //alternating functions for pins 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF2);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF2);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF2);
 

    //init timer 3
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    //enable clock for timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
     
    // PWM_frequency = timer_clk / (TIM_Period + 1)
    // TIM_Period = timer_clk / PWM_frequency - 1
    TIM_BaseStruct.TIM_Prescaler    = 0;
    TIM_BaseStruct.TIM_CounterMode  = TIM_CounterMode_Up;

    TIM_BaseStruct.TIM_Period               = PWM_PERIOD;
    TIM_BaseStruct.TIM_ClockDivision        = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter    = 0;
	 
    // Initialize TIM3
    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_Cmd(TIM3, ENABLE);
    


    //PWM output settings
    TIM_OCInitTypeDef TIM_OCStruct;
    
    // PWM mode 2 : clear on compare match
    // PWM mode 1 : set on compare match
    TIM_OCStruct.TIM_OCMode         = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState    = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity     = TIM_OCPolarity_Low;

    //HSA pwm, PC9, TIM3_CH4
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC4Init(TIM3, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    //HSB pwm, PC8, TIM3_CH3
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC3Init(TIM3, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    //HSC pwm, PC7, TIM3_CH2
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC2Init(TIM3, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


    //stop motor    
    this->set(0, 0, 0);

    //set enable to high, for all phases
    Gpio<TGPIOA, 8, GPIO_MODE_OUT> enable;
    enable = 1;
}

void PWMLeft::set(uint32_t a_pwm, uint32_t b_pwm, uint32_t c_pwm)
{
    TIM3->CCR4 = a_pwm;
    TIM3->CCR3 = b_pwm;
    TIM3->CCR2 = c_pwm;
}







void PWMRight::init()
{
    //right motor PWM controll pins 
    //PWMA      : PB8, TIM4_CH3
    //PWMB      : PB7, TIM4_CH2
    //PWMC      : PB6, TIM4_CH1
    //enable    : PB9

    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 

    //init pins
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Medium_Speed;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    //alternating functions for pins 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF2);
 

    //init timer 4
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    //enable clock for timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
     
    // PWM_frequency = timer_clk / (TIM_Period + 1)
    // TIM_Period = timer_clk / PWM_frequency - 1
    TIM_BaseStruct.TIM_Prescaler    = 0;
    TIM_BaseStruct.TIM_CounterMode  = TIM_CounterMode_Up;

    TIM_BaseStruct.TIM_Period               = PWM_PERIOD;
    TIM_BaseStruct.TIM_ClockDivision        = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter    = 0;
	 
    // Initialize TIM4
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    TIM_Cmd(TIM4, ENABLE);
    


    //PWM output settings
    TIM_OCInitTypeDef TIM_OCStruct;
    
    // PWM mode 2 : clear on compare match
    // PWM mode 1 : set on compare match 
    TIM_OCStruct.TIM_OCMode         = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState    = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity     = TIM_OCPolarity_Low;

    //HSA pwm, PC9, TIM4_CH4
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    //HSB pwm, PC8, TIM4_CH3
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    //HSC pwm, PC7, TIM4_CH2
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    //stop motor    
    this->set(0, 0, 0);

    //set enable to high, for all phases
    Gpio<TGPIOB, 9, GPIO_MODE_OUT> enable;
    enable = 1;
}

void PWMRight::set(uint32_t a_pwm, uint32_t b_pwm, uint32_t c_pwm)
{
    TIM4->CCR3 = a_pwm;
    TIM4->CCR2 = b_pwm;
    TIM4->CCR1 = c_pwm;
}