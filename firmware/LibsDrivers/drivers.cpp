#include "drivers.h"

Timer                       timer;
Terminal                    terminal;
ADC_driver                  adc;
TI2C<TGPIOD, 1, 2, 5>      i2c;
 
IRSensor     ir_sensor;
LineSensor   line_sensor;
Gyro         gyro_sensor;   

 
MotorControl motor_control;
 
void drivers_init()
{
    SetSysClock(SysClok216_8HSE);
 
    timer.init();
    
    terminal.init();
    i2c.init();
    //ir_sensor.init(); 
    //adc.init();
    //line_sensor.init();
    gyro_sensor.init(i2c);    
    
    motor_control.init();

    timer.delay_ms(100);
}

