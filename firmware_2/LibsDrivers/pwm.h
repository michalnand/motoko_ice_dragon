#ifndef _PWM_H_
#define _PWM_H_

//20kHz PWM, 216MHz/2 is timer clock source
#define PWM_FREQUENCY           ((uint32_t)20000)
//#define PWM_PERIOD              ((uint32_t)(216000000)/PWM_FREQUENCY - 1)
#define PWM_PERIOD              ((uint32_t)(216000000/2)/PWM_FREQUENCY - 1)


class PWM
{ 
    public:
        virtual void init()
        {

        }

        virtual void set(uint32_t a_pwm, uint32_t b_pwm, uint32_t c_pwm)
        {
            (void)a_pwm;
            (void)b_pwm;
            (void)c_pwm;
        }
};


class PWMLeft: public PWM
{   
    public:
        void init();
        void set(uint32_t a_pwm, uint32_t b_pwm, uint32_t c_pwm);
};

class PWMRight: public PWM
{   
    public:
        void init();
        void set(uint32_t a_pwm, uint32_t b_pwm, uint32_t c_pwm);
};

#endif