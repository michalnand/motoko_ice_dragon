#include <gyro.h>
#include <drivers.h>

#define LSM6DS33_ADDRESS              ((unsigned char)0xD4)

#define LSM6DS33_WHO_AM_I             ((unsigned char)0x0F)
#define LSM6DS33_WHO_AM_I_33_VALUE    ((unsigned char)0x69)


#define LSM6DS33_CTRL1_XL             ((unsigned char)0x10)
#define LSM6DS33_CTRL2_G              ((unsigned char)0x11)
#define LSM6DS33_CTRL3_C              ((unsigned char)0x12)
#define LSM6DS33_CTRL4_C              ((unsigned char)0x13)
#define LSM6DS33_CTRL5_C              ((unsigned char)0x14)
#define LSM6DS33_CTRL6_C              ((unsigned char)0x15)
#define LSM6DS33_CTRL7_G              ((unsigned char)0x16)
#define LSM6DS33_CTRL8_XL             ((unsigned char)0x17)
#define LSM6DS33_CTRL9_XL             ((unsigned char)0x18)
#define LSM6DS33_CTRL10_C             ((unsigned char)0x19)



#define LSM6DS33_STATUS_REG           ((unsigned char)0x1E)

#define LSM6DS33_OUT_TEMP_L           ((unsigned char)0x20)
#define LSM6DS33_OUT_TEMP_H           ((unsigned char)0x21)

#define LSM6DS33_OUTX_L_G             ((unsigned char)0x22)
#define LSM6DS33_OUTX_H_G             ((unsigned char)0x23)
#define LSM6DS33_OUTY_L_G             ((unsigned char)0x24)
#define LSM6DS33_OUTY_H_G             ((unsigned char)0x25)
#define LSM6DS33_OUTZ_L_G             ((unsigned char)0x26)
#define LSM6DS33_OUTZ_H_G             ((unsigned char)0x27)


#define LSM6DS33_OUTX_L_XL            ((unsigned char)0x28)
#define LSM6DS33_OUTX_H_XL            ((unsigned char)0x29)
#define LSM6DS33_OUTY_L_XL            ((unsigned char)0x2A)
#define LSM6DS33_OUTY_H_XL            ((unsigned char)0x2B)
#define LSM6DS33_OUTZ_L_XL            ((unsigned char)0x2C)
#define LSM6DS33_OUTZ_H_XL            ((unsigned char)0x2D)


Gyro *g_gyro_ptr;


#ifdef __cplusplus
extern "C" {
#endif

void TIM5_IRQHandler(void)
{ 
    g_gyro_ptr->callback();
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);  
} 
 
#ifdef __cplusplus
}
#endif


int Gyro::init(I2C_Interface &i2c_interface, int dt)
{
    terminal << "gyro_sensor init start\n";

    g_gyro_ptr              = this;

    this->i2c               = &i2c_interface;
    this->dt                = dt;
    this->measurement_id    = 0;



    //software reset
    i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL3_C,  (1<<0));

    timer.delay_ms(50);

    //i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL1_XL, 0x00);
    //i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL2_G,  0x00);

    if (i2c->read_reg(LSM6DS33_ADDRESS, LSM6DS33_WHO_AM_I) != LSM6DS33_WHO_AM_I_33_VALUE)    
    {
        terminal << "gyro init [ERROR]\n";
        return -1;
    } 

    //accelerometer config
    //ACC 200Hz filter bandwidth
    //+-2g scale
    //1000Hz output data rate
    i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL1_XL, (1<<7)|(1<<0));

    //gyroscope config
    //+-500dps range
    //416Hz output data rate
    //i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL2_G, (1<<6)|(1<<5)|(1<<2) );
     
    //gyroscope config
    //+-500dps range
    //1.66kHz output data rate
    i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL2_G, (1<<7)|(1<<2) );
 
    //auto increment register counter
    i2c->write_reg(LSM6DS33_ADDRESS, LSM6DS33_CTRL3_C,  (1<<2));

    timer.delay_ms(50);

   
    //meassure gyro offset
    int32_t calibration_iterations = 100;

    offset_z = 0;
    for (int32_t i = 0; i < calibration_iterations; i++)
    {
        offset_z+= read(); 
        timer.delay_ms(2); 
    } 

    offset_z  = offset_z/calibration_iterations;

    angular_rate_z = 0;
    angle_z        = 0;

    //init timer 5 interrupt for callback calling
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(dt*1000);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM5, ENABLE); 

    
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 


    terminal << "gyro_sensor init [DONE]\n";
    return 0;
}


void Gyro::callback()
{
    int32_t raw    = read() - offset_z; 

    //gyro in +-500dps range
    float tmp = (raw*500)/32768;
     
    //angular_rate = 1 means one rotation/s
    angular_rate_z = tmp/360.0; 
    angle_z        = angle_z + (dt*0.001)*angular_rate_z;

    measurement_id+= 1;
}

int32_t Gyro::read()
{
    i2c->start();
    i2c->write(LSM6DS33_ADDRESS);
    i2c->write(LSM6DS33_OUTZ_L_G); 

    i2c->start();
    i2c->write(LSM6DS33_ADDRESS|0x01);

    int16_t z;

    z = ((int16_t)i2c->read(1)) << 0;
    z|= ((int16_t)i2c->read(0)) << 8;

    i2c->stop();

    return z;
}

float Gyro::get_angular_rate()
{
    return angular_rate_z;
}

float Gyro::get_angle()
{
    return angle_z;
}