#include <stdint.h>
#include <adc_driver.h>
#include <gpio.h>

#include <drivers.h>



ADC_driver *g_adc_ptr;

#ifdef __cplusplus 
extern "C" {
#endif
 
void ADC_IRQHandler(void)
{
    //read value
    g_adc_ptr->adc_result[g_adc_ptr->adc_current_idx] = ADC_GetConversionValue(ADC1);
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    
    //move to next channel
    g_adc_ptr->adc_current_idx = (g_adc_ptr->adc_current_idx + 1)%ADC_CHANNELS_COUNT;

    //one complete measurement scan done, process results
    if (g_adc_ptr->adc_current_idx == 0)
    {
        g_adc_ptr->callback();
        g_adc_ptr->measurement_id++;
    }


    //trigger next masurement
    //ADC_RegularChannelConfig(ADC1, adc_channels[adc_current_idx], 1, ADC_SampleTime_15Cycles);
    //ADC_RegularChannelConfig(ADC1, g_adc_ptr->adc_channels[g_adc_ptr->adc_current_idx], 1, ADC_SampleTime_112Cycles); 
    ADC_RegularChannelConfig(ADC1, g_adc_ptr->adc_channels[g_adc_ptr->adc_current_idx], 1, ADC_SampleTime_480Cycles); 

    ADC_SoftwareStartConv(ADC1);
}
 
#ifdef __cplusplus
}
#endif

ADC_driver::ADC_driver()
{

}

void ADC_driver::init()
{
    //clear results
    adc_current_idx       = 0; 
    measurement_id= 0;
    g_adc_ptr          = this;

    for (unsigned int i = 0; i < ADC_CHANNELS_COUNT; i++)
    {
        adc_result[i] = 0;
    }

    //configure pins to analog input mode
    Gpio<TGPIOA, 0, GPIO_MODE_AN> adc_0;
    Gpio<TGPIOA, 1, GPIO_MODE_AN> adc_1;
    Gpio<TGPIOA, 2, GPIO_MODE_AN> adc_2;
    Gpio<TGPIOA, 3, GPIO_MODE_AN> adc_3;
    Gpio<TGPIOA, 4, GPIO_MODE_AN> adc_4;
    Gpio<TGPIOA, 5, GPIO_MODE_AN> adc_5;
    Gpio<TGPIOA, 6, GPIO_MODE_AN> adc_6;
    Gpio<TGPIOA, 7, GPIO_MODE_AN> adc_7;

    
    Gpio<TGPIOB, 1, GPIO_MODE_AN>   ir_left;
    Gpio<TGPIOC, 5, GPIO_MODE_AN>   ir_front_left;
    Gpio<TGPIOC, 4, GPIO_MODE_AN>   ir_front_right;
    Gpio<TGPIOB, 0, GPIO_MODE_AN>   ir_right;



    //channels mapping
    adc_channels[0]   = ADC_LINE_0;
    adc_channels[1]   = ADC_LINE_1;
    adc_channels[2]   = ADC_LINE_2;
    adc_channels[3]   = ADC_LINE_3;
    adc_channels[4]   = ADC_LINE_4;
    adc_channels[5]   = ADC_LINE_5;
    adc_channels[6]   = ADC_LINE_6;
    adc_channels[7]   = ADC_LINE_7;

    adc_channels[8]   = ADC_IR_LEFT;
    adc_channels[9]   = ADC_IR_FRONT_LEFT;
    adc_channels[10]  = ADC_IR_FRONT_RIGHT;
    adc_channels[11]  = ADC_IR_RIGHT;


    //ADC init
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    ADC_CommonInitStruct.ADC_Mode               =   ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler          =   ADC_Prescaler_Div8; 
    ADC_CommonInitStruct.ADC_DMAAccessMode      =   ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay   =   ADC_TwoSamplingDelay_5Cycles;

    ADC_CommonInit(&ADC_CommonInitStruct);


    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_ScanConvMode         =   DISABLE; 
    ADC_InitStruct.ADC_Resolution           =   ADC_Resolution_12b;
    ADC_InitStruct.ADC_ContinuousConvMode   =   DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConv     =   ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStruct.ADC_ExternalTrigConvEdge =   ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign            =   ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion      =   1;

    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_RegularChannelConfig(ADC1, adc_channels[0], 1, ADC_SampleTime_112Cycles);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    //configure and enable ADC1 interrupt
    NVIC_InitTypeDef         NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel                      = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //start first conversion
    ADC_SoftwareStartConv(ADC1);
} 

uint16_t* ADC_driver::get()
{
    return (uint16_t*)adc_result;
}
 

void ADC_driver::callback()
{
    ir_sensor.callback();
} 