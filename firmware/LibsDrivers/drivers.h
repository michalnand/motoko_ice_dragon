#ifndef _DRIVERS_H_
#define _DRIVERS_H_

#include <device.h>

#include <clock.h>
#include <gpio.h>
#include <timer.h>
#include <terminal.h>
#include <i2c.h>
#include <adc_driver.h>


#include <ir_sensor.h>
#include <line_sensor.h>
#include <gyro.h> 

#include <motor_control.h>

extern  Timer                       timer;
extern  Terminal                    terminal;
extern  ADC_driver                  adc;
extern  TI2C<TGPIOD, 1, 2, 5>       i2c;

extern  IRSensor     ir_sensor;
extern  LineSensor   line_sensor;
extern  Gyro         gyro_sensor;

extern  MotorControl motor_control;

void drivers_init();

#endif