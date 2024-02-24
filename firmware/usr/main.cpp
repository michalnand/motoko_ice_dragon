#include "device.h"
#include <drivers.h>
#include <tests.h>

#include <pwm.h>

#include <identification.h>
#include <position_control_lqr.h>
#include <position_control_lqg.h>

#include <fmath.h>
 
#define LED_1_GPIO        TGPIOE
#define LED_1_PIN         3
 
#define LED_2_GPIO        TGPIOE 
#define LED_2_PIN         2   

#define LED_3_GPIO        TGPIOD 
#define LED_3_PIN         13

#define KEY_GPIO          TGPIOE 
#define KEY_PIN           4 


void brick_avoid(PositionControlLQR &position_control, float a, float d, float angle)
{
  float d_curr = position_control.distance;
  float a_curr = position_control.angle;

  float points_distance[] = {0.0,    a,   0.0,   d,  0.0,  a,   0.0};
  float points_angle[]    = {angle, 0.0, -angle, 0.0, -angle, 0.0, angle}; 


  for (unsigned int n = 0; n < 7; n++)
  {
    d_curr+= points_distance[n];
    a_curr+= points_angle[n]*PI/180.0;

    position_control.set(d_curr, a_curr);

    while (abs(d_curr - position_control.distance) > 5.0)
    {
      __asm("nop");
    }

    while (abs(a_curr - position_control.angle) > 5.0*PI/180.0)
    {
      __asm("nop");
    }
  }
}



void broken_line_search(PositionControlLQR &position_control, float d, float angle)
{
  float d_curr = position_control.distance;
  float a_curr = position_control.angle;

  float points_distance[] = {d,     -d,     d,      -d,     d};
  float points_angle[]    = {angle, -angle, -angle, angle,  0.0}; 


  for (unsigned int n = 0; n < 5; n++)
  {
    d_curr+= points_distance[n];
    a_curr+= points_angle[n]*PI/180.0;

    position_control.set(d_curr, a_curr);

    while (abs(d_curr - position_control.distance) > 5.0)
    {
      __asm("nop");
    }

    while (abs(a_curr - position_control.angle) > 5.0*PI/180.0)
    {
      __asm("nop");
    }
  }
}


void line_followingA(PositionControlLQR &position_control, float speed)
{
  float line_position     = 0.0;
  float forward_position  = 0.0;

  while (1)
  {
    if (line_sensor.line_lost_type == LINE_LOST_NONE)
    {
      line_position    = position_control.angle + line_sensor.left_angle;
      forward_position = position_control.distance + speed;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_LEFT)
    {
      line_position    = position_control.angle + 90.0*PI/180.0;
      forward_position = position_control.distance;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_RIGHT)
    {
      line_position    = position_control.angle - 90.0*PI/180.0;
      forward_position = position_control.distance;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_CENTER)
    {
      return; 
    } 


    position_control.set(forward_position, line_position);
    timer.delay_ms(4);
  }
} 



void line_followingB(PositionControlLQR &position_control, float speed_min, float speed_max)
{
  float line_position     = 0.0;
  float forward_position  = 0.0;

  float speed_curr = 0.0;

  float ramp_up   =  10.0;   
  float ramp_down = -1.5*ramp_up;       
   
  while (1)    
  {
    if (line_sensor.line_lost_type == LINE_LOST_NONE)
    {
      line_position    = position_control.angle + line_sensor.left_angle; 

      float quality = 1.0 - 1.0*max(abs(line_sensor.left_position), abs(line_sensor.right_position));
      quality = clip(quality, 0.0, 1.0);  

      speed_curr = speed_curr + (1.0 - quality)*ramp_down + quality*ramp_up;

      speed_curr = clip(speed_curr, speed_min, speed_max);
      
      forward_position = position_control.distance + speed_curr;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_LEFT) 
    {
      line_position    = position_control.angle + 80.0*PI/180.0;
      forward_position = position_control.distance + speed_min/2;
      speed_curr       = speed_min/2;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_RIGHT)  
    {
      line_position    = position_control.angle - 80.0*PI/180.0;
      forward_position = position_control.distance + speed_min/2;
      speed_curr       = speed_min/2;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_CENTER)
    {
      return; 
    } 


    position_control.set(forward_position, line_position);
    timer.delay_ms(4);
  }
} 






     


    
int main(void)      
{ 
  drivers_init();
  
  PositionControlLQR position_control;
  position_control.init();
  position_control.set(0.0, 0.0);

  terminal << "\n\n\n"; 
  terminal << "machine ready\n";



  Gpio<LED_1_GPIO, LED_1_PIN, GPIO_MODE_OUT> led_1;   //user led
  led_1 = 1;  

  Gpio<LED_2_GPIO, LED_2_PIN, GPIO_MODE_OUT> led_2;   //bottom led
  Gpio<LED_3_GPIO, LED_3_PIN, GPIO_MODE_OUT> led_3;   //bottom led
    
  Gpio<KEY_GPIO, KEY_PIN, GPIO_MODE_IN_PULLUP> key;   //user button
 
  for (unsigned int i = 0; i < 5; i++)
  {     
    led_2 = 1;  
    led_3 = 1; 
    timer.delay_ms(50);
 
    led_2 = 0; 
    led_3 = 0; 
    timer.delay_ms(100);
  }

  led_2 = 1; 
  led_3 = 1; 


  while (key != 0)
  { 
    led_1 = 1; 
    timer.delay_ms(100);

    led_1 = 0; 
    timer.delay_ms(200);
  }
 
  led_1 = 1; 

  while (key == 0)
  {
    led_1 = 1; 
    timer.delay_ms(50);

    led_1 = 0; 
    timer.delay_ms(50);
  } 

  timer.delay_ms(1500);

  
  

  //timer_test();

  //left_motor_connect_test();
  //right_motor_connect_test();

  //ir_sensor_test();
  //line_sensor_test();
  //gyro_sensor_test(); 
  //encoder_sensor_test();
  //sensors_test();
  

  //left_motor_pwm_test();
  //right_motor_pwm_test();
  //encoder_sensor_test();

  //motor_identification();
  //motor_driver_test();  
  //smooth_motor_driver_test();
  //robot_dynamics_identification();

  //line_stabilise(); 
  //gyro_turn_test(); 




  //line_followingA(position_control, 100.0);
  line_followingB(position_control, 100.0, 200.0);




  while (1) 
  {
    led_1 = 1; 
    timer.delay_ms(100);

    led_1 = 0; 
    timer.delay_ms(100);

    led_1 = 1; 
    timer.delay_ms(200);

    led_1 = 0; 
    timer.delay_ms(400);
  }
 
  return 0;
} 