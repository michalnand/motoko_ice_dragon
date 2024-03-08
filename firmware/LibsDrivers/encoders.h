#ifndef _ENCODERS_H_
#define _ENCODERS_H_

#include <device.h>

#define I2C_ADDRESS         ((unsigned char)0x36<<1)

#define ZMCO_ADR            ((unsigned char)0x00)
#define ZPOS_H_ADR          ((unsigned char)0x01)
#define ZPOS_L_ADR          ((unsigned char)0x02)
#define MPOS_H_ADR          ((unsigned char)0x03)
#define MPOS_L_ADR          ((unsigned char)0x04)
#define MANG_H_ADR          ((unsigned char)0x05)
#define MANG_L_ADR          ((unsigned char)0x06)
#define CONF_H_ADR          ((unsigned char)0x07) 
#define CONF_L_ADR          ((unsigned char)0x08)

#define RAW_ANGLE_H_ADR     ((unsigned char)0x0C)
#define RAW_ANGLE_L_ADR     ((unsigned char)0x0D)
#define ANGLE_H_ADR         ((unsigned char)0x0E)
#define ANGLE_L_ADR         ((unsigned char)0x0F)

#define STATUS_ADR          ((unsigned char)0x0B)
#define AGC_ADR             ((unsigned char)0x1A)
#define MAGNITUDE_H_ADR     ((unsigned char)0x1B)
#define MAGNITUDE_L_ADR     ((unsigned char)0x1C)


template <unsigned char left_port_name, unsigned char left_sda_pin, unsigned char left_scl_pin, unsigned char right_port_name, unsigned char right_sda_pin, unsigned char right_scl_pin, unsigned int bus_speed = 20>
class ENCODERS
{
    //left encoder
    public:
        int32_t left_angle; 
        int32_t left_position;
        int32_t left_angular_velocity;

    private:
        int32_t left_position_prev;
        int32_t left_prev_value;


    //right encoder
    public:
        int32_t right_angle; 
        int32_t right_position;
        int32_t right_angular_velocity;

    private:
        int32_t right_position_prev;
        int32_t right_prev_value;


    /*
    int init()
    {
        this->left_angle            = 0;
        this->left_position         = 0;
        this->left_angular_velocity = 0;

        this->left_position_prev    = 0;
        this->left_prev_value       = 0;


        this->right_angle            = 0;
        this->right_position         = 0;
        this->right_angular_velocity = 0;

        this->right_position_prev    = 0;
        this->right_prev_value       = 0;


        i2c_init();

        //check if device responds using ACK, since sensor doesn't have WHO_AM_I ID
        unsigned char ack = i2c_check(I2C_ADDRESS);
        if (ack == 0)
        {
            return -1;
        }  
  
        //power on
        //hysteresis 1 LSB 
        //slow filter only, 8x 
        i2c_write_reg(I2C_ADDRESS, CONF_L_ADR, (1<<2));
        i2c_write_reg(I2C_ADDRESS, CONF_H_ADR, (1<<0)); 
        


        //set zero angle
        set_zero();
        set_zero();

        this->read_angle();
        this->update(1000);


        this->position          = this->angle;
        this->position_prev     = this->angle;
        this->angular_velocity  = 0;
        this->prev_value        = 0;
        
        return 0;
    }
    */
};

#endif