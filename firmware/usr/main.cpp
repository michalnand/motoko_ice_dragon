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

#include <matrix.h> 

#include <lqg.h>     
     
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

  //line_follow_test();


  //motor_control.set_torque(0, 1000);

  

  /*
  //turn test
  float req_angle[] = {0.0, 90.0, 0.0, -90.0, 0.0, 90.0, 0.0, -90.0, 0.0, 90.0, 0.0, -90.0};


  for (unsigned int n = 0; n < 12; n++)
  {
    float dist  = 0;
    float angle = req_angle[n]*PI/180.0;

    position_control.set(dist, angle);

    timer.delay_ms(300);
  } 

  position_control.set(0, 0);
  */


  
    //forward test
    float req_distance[] = {0.0, 100.0, 0.0, 100.0, 0.0, 100.0, 0.0, 100.0};

 
    for (unsigned int n = 0; n < 8; n++) 
    {
      float dist  = req_distance[n];
      float angle = 0.0;

      position_control.set(dist, angle);

      timer.delay_ms(300);
    } 

    position_control.set(0, 0);
  

  /*
  float req_dist[]  = {0.0, 80.0, 80.0,  80.0, 0.0,  80.0, 0.0,  80.0};
  float req_angle[] = {0.0, 0.0, 90.0, 0.0, 90.0, 0.0, 90.0, 0.0};

  float dist_sum = 0.0;
  float angle_sum = 0.0;

  while (1)
  {
    for (unsigned int n = 0; n < 8; n++)
    {
      dist_sum+= req_dist[n];
      angle_sum+= req_angle[n]*PI/180.0;

      position_control.set(dist_sum, angle_sum);

      timer.delay_ms(1000);
    }
  }
  */

 

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