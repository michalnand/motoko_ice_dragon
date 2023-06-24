#include <gyro.h>
#include <drivers.h>


//#define LSM6DS0_ADDRESS              ((unsigned char)0xD6)
#define LSM6DS0_ADDRESS              ((unsigned char)0xD4)

#define LSM6DS0_WHO_AM_I             ((unsigned char)0x0F)
#define LSM6DS0_WHO_AM_I_VALUE       ((unsigned char)0x68)
#define LSM6DS0_WHO_AM_I_33_VALUE    ((unsigned char)0x69)

#define LSM6DS0_CTRL_REG1_G         ((unsigned char)0x10)
#define LSM6DS0_CTRL_REG2_G         ((unsigned char)0x11)
#define LSM6DS0_CTRL_REG3_G         ((unsigned char)0x12)
#define LSM6DS0_CTRL_REG4_G         ((unsigned char)0x1E)
#define LSM6DS0_CTRL_REG10_G        ((unsigned char)0x24)

#define LSM6DS0_GYRO_XOUT_L         ((unsigned char)0x18)
#define LSM6DS0_GYRO_XOUT_H         ((unsigned char)0x19)
#define LSM6DS0_GYRO_YOUT_L         ((unsigned char)0x1A)
#define LSM6DS0_GYRO_YOUT_H         ((unsigned char)0x1B)
#define LSM6DS0_GYRO_ZOUT_L         ((unsigned char)0x1C)
#define LSM6DS0_GYRO_ZOUT_H         ((unsigned char)0x1D)






#define LSM6DS0_FIFO_CTR	           ((unsigned char)0x2E)


#define LSM6DS0_G_ODR_PD                                ((unsigned char)0x00) /*!< Output Data Rate: Power-down*/
#define LSM6DS0_G_ODR_14_9HZ                            ((unsigned char)0x20) /*!< Output Data Rate: 14.9 Hz, cutoff 5Hz */
#define LSM6DS0_G_ODR_59_5HZ                            ((unsigned char)0x40) /*!< Output Data Rate: 59.5 Hz, cutoff 19Hz */
#define LSM6DS0_G_ODR_119HZ                             ((unsigned char)0x60) /*!< Output Data Rate: 119 Hz, cutoff 38Hz*/
#define LSM6DS0_G_ODR_238HZ                             ((unsigned char)0x80) /*!< Output Data Rate: 238 Hz, cutoff 76Hz*/
#define LSM6DS0_G_ODR_476HZ                             ((unsigned char)0xA0) /*!< Output Data Rate: 476 Hz, cutoff 100Hz*/
#define LSM6DS0_G_ODR_952HZ                             ((unsigned char)0xC0)

#define LSM6DS0_G_FS_245                               ((unsigned char)0x00) /*!< Full scale: 245 dps*/
#define LSM6DS0_G_FS_245_MDPS                          ((int32_t)875)            /*!< 8.75mdps*/

#define LSM6DS0_G_FS_500                               ((unsigned char)0x08) /*!< Full scale: 500 dps */
#define LSM6DS0_G_FS_500_MDPS                          ((int32_t)1750)            /*!< 17.5mdps*/

#define LSM6DS0_G_FS_2000                              ((unsigned char)0x18) /*!< Full scale: 2000 dps */
#define LSM6DS0_G_FS_2000_MDPS                         ((int32_t)7000)            /*!< 70mdps*/


#define LSM6DS0_G_XEN_ENABLE                           ((unsigned char)0x08)
#define LSM6DS0_G_YEN_ENABLE                           ((unsigned char)0x10)
#define LSM6DS0_G_ZEN_ENABLE                           ((unsigned char)0x20)


#define LSM6DS0_XG_CTRL_REG6_XL                        ((unsigned char)0x20)
#define LSM6DS0_XG_CTRL_REG5_XL                        ((unsigned char)0x1F)
#define LSM6DS0_XG_OUT_X_L_XL                          ((unsigned char)0x28)
#define LSM6DS0_XG_OUT_X_H_XL                          ((unsigned char)0x29)
#define LSM6DS0_XG_OUT_Y_L_XL                          ((unsigned char)0x2A)
#define LSM6DS0_XG_OUT_Y_H_XL                          ((unsigned char)0x2B)
#define LSM6DS0_XG_OUT_Z_L_XL                          ((unsigned char)0x2C)
#define LSM6DS0_XG_OUT_Z_H_XL                          ((unsigned char)0x2D)


#define LSM6DS0_XL_ODR_10HZ                             ((uint8_t)0x20) /*!< Output Data Rate: 10 Hz*/
#define LSM6DS0_XL_ODR_50HZ                             ((uint8_t)0x40) /*!< Output Data Rate: 50 Hz */
#define LSM6DS0_XL_ODR_119HZ                            ((uint8_t)0x60) /*!< Output Data Rate: 119 Hz */
#define LSM6DS0_XL_ODR_238HZ                            ((uint8_t)0x80) /*!< Output Data Rate: 238 Hz */
#define LSM6DS0_XL_ODR_476HZ                            ((uint8_t)0xA0) /*!< Output Data Rate: 476 Hz */
#define LSM6DS0_XL_ODR_952HZ                            ((uint8_t)0xC0) /*!< Output Data Rate: 952 Hz */

#define LSM6DS0_XL_ODR_MASK                             ((uint8_t)0xE0)


#define LSM6DS0_XL_FS_2G                                ((uint8_t)0x00) /*!< Full scale: +- 2g */
#define LSM6DS0_XL_FS_4G                                ((uint8_t)0x10) /*!< Full scale: +- 4g */
#define LSM6DS0_XL_FS_8G                                ((uint8_t)0x18) /*!< Full scale: +- 8g */


#define LSM6DS0_XL_ZEN_ENABLE                           ((uint8_t)0x20)
#define LSM6DS0_XL_YEN_ENABLE                           ((uint8_t)0x10)
#define LSM6DS0_XL_XEN_ENABLE                           ((uint8_t)0x08)



Gyro *g_gyro_ptr;


#ifdef __cplusplus
extern "C" {
#endif

void TIM6_DAC_IRQHandler(void)
{ 
    g_gyro_ptr->callback();
    TIM_ClearITPendingBit(TIM6, TIM_IT_CC1);  
} 
 
#ifdef __cplusplus
}
#endif


int Gyro::init(I2C_Interface &i2c_interface, int dt)
{
    this->i2c           = &i2c_interface;
    this->dt            = dt;
    this->measurement_id   = 0;
    g_gyro_ptr          = this;

    if ((i2c->read_reg(LSM6DS0_ADDRESS, LSM6DS0_WHO_AM_I) != LSM6DS0_WHO_AM_I_VALUE) &&
        (i2c->read_reg(LSM6DS0_ADDRESS, LSM6DS0_WHO_AM_I) != LSM6DS0_WHO_AM_I_33_VALUE) )
    {
        //return -1;
    }

    i2c->write_reg(LSM6DS0_ADDRESS, LSM6DS0_CTRL_REG1_G, LSM6DS0_G_ODR_238HZ | LSM6DS0_G_FS_500 );
    i2c->write_reg(LSM6DS0_ADDRESS, LSM6DS0_CTRL_REG4_G, LSM6DS0_G_XEN_ENABLE|LSM6DS0_G_YEN_ENABLE|LSM6DS0_G_ZEN_ENABLE); // anable x, y, z axis
    i2c->write_reg(LSM6DS0_ADDRESS, LSM6DS0_FIFO_CTR, 0); //bypass mode
    i2c->write_reg(LSM6DS0_ADDRESS, LSM6DS0_CTRL_REG3_G, 0); //hp filter disable

    i2c->write_reg(LSM6DS0_ADDRESS, LSM6DS0_XG_CTRL_REG6_XL, LSM6DS0_XL_ODR_119HZ | LSM6DS0_XL_FS_2G); //output data rate, full scale
    i2c->write_reg(LSM6DS0_ADDRESS, LSM6DS0_XG_CTRL_REG5_XL, LSM6DS0_XL_XEN_ENABLE|LSM6DS0_XL_YEN_ENABLE|LSM6DS0_XL_ZEN_ENABLE);

    sensitivity = LSM6DS0_G_FS_500_MDPS;

    
    timer.delay_ms(100);

    //meassure gyro offset
    
    offset_z = 0;
    
    unsigned int calibration_iterations = 256;

    for (unsigned int i = 0; i < calibration_iterations; i++)
    {
        offset_z+= read();
        timer.delay_ms(2);
    }

    offset_z       = offset_z/calibration_iterations;

    angular_rate_z = 0;
    angle_z        = 0;



    //init timer 6 interrupt for callback calling
    
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = (216*1000*dt);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM6, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM6, ENABLE);  

    //TIM6_DAC_IRQHandler
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    return 0;
}


void Gyro::callback()
{
    int32_t raw    = read() - offset_z;
    angular_rate_z = (LSM6DS0_G_FS_500_MDPS*raw)/1000;
    angle_z        = angle_z + angular_rate_z;
    measurement_id+= 1;
}

int Gyro::read()
{
    i2c->start();
    i2c->write(LSM6DS0_ADDRESS);
    i2c->write(LSM6DS0_GYRO_ZOUT_H);

    i2c->start();
    i2c->write(LSM6DS0_ADDRESS|0x01);

    int16_t z;

    z = ((int16_t)i2c->read(1)) << 8;
    z|= ((int16_t)i2c->read(0)) << 0;

    i2c->stop();

    return z;
}

int32_t Gyro::get_angular_rate()
{
    return angular_rate_z;
}

int32_t Gyro::get_angle()
{
    return angle_z;
}