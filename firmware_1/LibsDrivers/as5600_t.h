#ifndef _AS5600T_H_
#define _AS5600T_H_

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

#define ENCODER_RESOLUTION  ((int32_t)4096)

template <unsigned char port_name, unsigned char sda_pin, unsigned char scl_pin, unsigned int bus_speed = 20>
class AS5600T
{
    public:
        int32_t angle; 

        int32_t position;
        int32_t angular_velocity;
        int32_t position_fil;

    private:
        int32_t position_prev;
        int32_t prev_value;

    public:
        AS5600T()
        {

        }

        ~AS5600T()
        { 

        }

        int init()
        { 
            i2c_init();

            this->angle         = 0; 

            //check if device responds using ACK, since sensor doesn't have WHO_AM_I ID
            unsigned char ack = i2c_check(I2C_ADDRESS);
            if (ack == 0)
            {
                return -1;
            }  

            
           
            //power on
            //hysteresis 1 LSB 
            //slow filter only, 8x 
            //i2c_write_reg(I2C_ADDRESS, CONF_L_ADR, (1<<2));
            //i2c_write_reg(I2C_ADDRESS, CONF_H_ADR, (1<<0)); 


            //hysteresis 2 LSB 
            //slow filter only, 8x 
            i2c_write_reg(I2C_ADDRESS, CONF_L_ADR, (1<<3));
            i2c_write_reg(I2C_ADDRESS, CONF_H_ADR, (1<<0)); 
            
 
            this->position          = 0;
            this->position_prev     = 0;
            this->angular_velocity  = 0;
            this->prev_value        = 0;

            this->position_fil      = 0;

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

        int32_t read_angle()
        {
            this->angle = i2c_read_reg_16bit(I2C_ADDRESS, ANGLE_H_ADR)&0x0fff;
            return this->angle;
        }


        void update(int32_t dt_us)
        {
            int16_t value = i2c_read_reg_16bit(I2C_ADDRESS, ANGLE_H_ADR)&0x0fff;

            this->angle = value;
         
            this->position_prev = this->position; 
        
            //  whole rotation CW?
            //  less than half a circle
            if ((this->prev_value > 2048) && ( value < (this->prev_value - 2048)))
            {
                this->position = this->position + 4096 - this->prev_value + value;
            } 
        
            //  whole rotation CCW?
            //  less than half a circle
            else if ((value > 2048) && ( this->prev_value < (value - 2048)))
            {
                this->position = this->position - 4096 - this->prev_value + value;
            }
            else 
            {
                this->position = this->position - this->prev_value + value;
            }


            int32_t tmp = ((this->position - this->position_prev)*1000000)/dt_us;
            
            //complementary LP filter
            this->position_fil     = (7*this->position_fil + 1*this->position)/8;  
            this->angular_velocity = (7*this->angular_velocity + 1*tmp)/8;      
                
            this->prev_value       = value;
        }
 



        void set_zero()
        {
            //set zero angle
            uint8_t raw_h = i2c_read_reg(I2C_ADDRESS, RAW_ANGLE_H_ADR);
            uint8_t raw_l = i2c_read_reg(I2C_ADDRESS, RAW_ANGLE_L_ADR);

            i2c_write_reg(I2C_ADDRESS, ZPOS_H_ADR, raw_h); 
            i2c_write_reg(I2C_ADDRESS, ZPOS_L_ADR, raw_l); 
        }


    private:
    
        inline void i2c_SetHighSCL()  
        {
            scl.set_mode(GPIO_MODE_IN_FLOATING);
        }

        inline void i2c_SetLowSCL()   
        {
            scl.set_mode(GPIO_MODE_OUT);
        }

        inline void i2c_SetHighSDA()  
        {
            sda.set_mode(GPIO_MODE_IN_FLOATING);
        }

        inline void i2c_SetLowSDA()   
        {
            sda.set_mode(GPIO_MODE_OUT);
        }

    private:

        void i2c_init()
        {
            sda.init();
            scl.init();
            sda = 0;
            scl = 0;
        }


        inline void i2c_start()
        {
            i2c_SetHighSCL();
            i2c_SetHighSDA();

            i2c_SetHighSCL();
            i2c_SetLowSDA();

            i2c_SetLowSCL();
            i2c_SetHighSDA();

            i2c_delay();
        }

        inline void i2c_stop()
        {
            i2c_SetLowSCL();
            i2c_SetLowSDA();

            i2c_SetHighSCL();
            i2c_SetLowSDA();

            i2c_SetHighSCL();
            i2c_SetHighSDA();

            i2c_delay();
        }


        unsigned char i2c_write(unsigned char b)
        {
            unsigned char  i;
            unsigned char  return_ack;

            for (i = 0; i < 8; i++)
            {
                i2c_SetLowSCL();
                i2c_delay();

                if (b & (1<<7))
                {
                    i2c_SetHighSDA();
                }
                else
                {
                    i2c_SetLowSDA();
                }

                i2c_delay();
                i2c_SetHighSCL();

                i2c_delay();
                b <<= 1;
            }

            i2c_SetLowSCL();
            i2c_delay();
            i2c_SetHighSDA();
            i2c_delay();

            i2c_SetHighSCL();
            i2c_delay();

            if (sda)
                return_ack = NO_I2C_ACK;
            else
                return_ack = OK_I2C_ACK;

            i2c_SetLowSCL();
            i2c_delay();

            return(return_ack);
        }

        unsigned char i2c_read(unsigned char ack = 0)
        {
            unsigned char  i;
            unsigned char  c = 0x00;

            i2c_SetHighSDA();
            i2c_SetLowSCL();
            i2c_delay();

            for (i = 0; i < 8; i++)
            {
                c = c << 1;
                i2c_SetHighSCL();
                i2c_delay();

                if (sda)
                c = c | 0x01;

                i2c_SetLowSCL();
                i2c_delay();
            }


            if(ack)
            {
                i2c_SetLowSDA();
            }
            else
            {
                i2c_SetHighSDA();
            }

            i2c_delay();
            i2c_SetHighSCL();
            i2c_delay();
            i2c_SetLowSCL();
            i2c_delay();
            i2c_SetHighSDA();
            i2c_delay();

            return (c);
        }

        void i2c_delay()
        {
            volatile uint32_t loops = bus_speed;
            while (loops--)
            {

            }
        }


    private:

        void i2c_write_reg(unsigned char dev_adr, unsigned char reg_adr, unsigned char value)
        {
            i2c_start();
            i2c_write(dev_adr);  //slave address, write command
            i2c_write(reg_adr);  //send reg address
            i2c_write(value);
            i2c_stop();
        }

        unsigned char i2c_read_reg(unsigned char dev_adr, unsigned char reg_adr)
        {
            unsigned char res;

            i2c_start();
            i2c_write(dev_adr);  // slave address, write command
            i2c_write(reg_adr);  // send reg address

            i2c_start();
            i2c_write(dev_adr|0x01); // slave address, read command
            res = i2c_read(0);   // read data
            i2c_stop();

            return res;
        }

        unsigned int i2c_read_reg_16bit(unsigned char dev_adr, unsigned char reg_adr)
        {
            unsigned int result;

            i2c_start();
            i2c_write(dev_adr);  // slave address, write command
            i2c_write(reg_adr);  // send reg address
            i2c_start();
            
            i2c_write(dev_adr|0x01); // slave address, read command
            result = ((unsigned int)i2c_read(1))<<8;   // read data
            result|= ((unsigned int)i2c_read(0));
            i2c_stop();

            return result;
        }

        unsigned char i2c_check(unsigned char dev_adr)
        {
            i2c_start();
            unsigned char ack_res = i2c_write(dev_adr);
            i2c_stop();  

            return ack_res;
        } 


    private:
        Gpio<port_name, sda_pin, GPIO_MODE_IN_FLOATING>     sda;
        Gpio<port_name, scl_pin, GPIO_MODE_IN_FLOATING>     scl;
};


#endif