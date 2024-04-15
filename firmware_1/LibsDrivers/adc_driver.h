#ifndef _ADC_DRIVER_H_
#define _ADC_DRIVER_H_ 

#define ADC_CHANNELS_COUNT  ((uint32_t)(8 + 4))

#define LINE_SENSOR_OFFSET      ((uint32_t)(0))
#define IR_SENSOR_OFFSET        ((uint32_t)(8))


//sensors physical channels
#define ADC_LINE_0                 ((unsigned int)ADC_Channel_0)    //PA0
#define ADC_LINE_1                 ((unsigned int)ADC_Channel_1)    //PA1
#define ADC_LINE_2                 ((unsigned int)ADC_Channel_2)    //PA2
#define ADC_LINE_3                 ((unsigned int)ADC_Channel_3)    //PA3
#define ADC_LINE_4                 ((unsigned int)ADC_Channel_4)    //PA4
#define ADC_LINE_5                 ((unsigned int)ADC_Channel_5)    //PA5
#define ADC_LINE_6                 ((unsigned int)ADC_Channel_6)    //PA6
#define ADC_LINE_7                 ((unsigned int)ADC_Channel_7)    //PA7


#define ADC_IR_LEFT                 ((unsigned int)ADC_Channel_9)    //PB1
#define ADC_IR_FRONT_LEFT           ((unsigned int)ADC_Channel_15)   //PC5
#define ADC_IR_FRONT_RIGHT          ((unsigned int)ADC_Channel_14)   //PC4
#define ADC_IR_RIGHT                ((unsigned int)ADC_Channel_8)    //PB0



  
class ADC_driver
{
    public:
        ADC_driver();

        void init();
        uint16_t* get();

        void callback();

    public:    
        uint32_t adc_current_idx;
        uint16_t adc_channels[ADC_CHANNELS_COUNT];
        uint16_t adc_result[ADC_CHANNELS_COUNT];

        uint32_t measurement_id;
};


#endif
