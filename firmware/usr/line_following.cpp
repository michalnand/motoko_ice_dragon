#include "line_following.h"
#include <fmath.h>


LineFollowing::LineFollowing(PositionControlLQR &position_control)
{
    this->position_control = &position_control;

    this->r_min = 80.0;
    this->r_max = 10000.0;

    this->speed_min = 150.0;
    this->speed_max = 350.0;


    this->q_penalty = 1.5;
    this->qr_max    = 6.0;
    this->qr_min    = 2.0;

    quality_filter.init(1.0);
}


int LineFollowing::main()
{
    float speed_min_curr = 0.0;

    while (1)
    {
        position_control->enable_lf();

        speed_min_curr = clip(speed_min_curr + speed_min/100.0, 0.0, speed_min);

        float position = line_sensor.left_position; 

        quality_filter.step(abs(position));

        float radius  = this->estimate_turn_radius(position, 1.0/r_max);
        radius  = -sgn(position)*clip(radius, r_min, r_max);      

        float q = 1.0 - this->q_penalty*quality_filter.max(); 

        q = clip(q, 0.0, 1.0);         

        //if quality is high (close to 1), increase radius - allows faster speed
        float kr = q*this->qr_max + (1.0 - q)*this->qr_min;  

        //if quality is high (close to 1), use higher speed
        float speed = q*speed_max + (1.0 - q)*speed_min_curr;  

        position_control->set_circle_motion(kr*radius, speed);

        timer.delay_ms(4);

        /*
        if (line_sensor.line_lost_type != LINE_LOST_NONE)   
        {
            break; 
        }  
        */  

        if (line_sensor.line_lost_type == LINE_LOST_CENTER)   
        {
            break; 
        }  
    } 

    position_control->disable_lf();
    position_control->set(position_control->distance, position_control->angle); 

    return 0;
}

void LineFollowing::line_search()
{
    //TODO

    float d = 0.0;
    float angle = 0.0;

    float d_curr = position_control->distance;
    float a_curr = position_control->angle;

    float points_distance[] = {d,     -d,     d,      -d,     d};
    float points_angle[]    = {angle, -angle, -angle, angle,  0.0}; 


    for (unsigned int n = 0; n < 5; n++)
    {
        d_curr+= points_distance[n];
        a_curr+= points_angle[n]*PI/180.0;

        position_control->set(d_curr, a_curr);

        while (abs(d_curr - position_control->distance) > 5.0)
        {
        __asm("nop");
        }

        while (abs(a_curr - position_control->angle) > 5.0*PI/180.0)
        {
        __asm("nop");
        }
    }
}

float LineFollowing::estimate_turn_radius(float sensor_reading, float eps)
{
  float x = SENSORS_DISTANCE;
  float y = 0.5*SENSORS_BRACE*abs(sensor_reading);

  float r = (y*y + x*x)/(2.0*y + eps);

  return r;
}










/*
void line_followingA(PositionControlLQR &position_control, float r_min, float r_max, float speed_min, float speed_max)
{
  float radius  = 0.0;
  float speed   = 0.0;
  
  float ramp = 4;    

  while (1)
  {
    if (line_sensor.line_lost_type == LINE_LOST_NONE)
    { 
      float pos = line_sensor.left_position; 
      float ws  = clip(1.5*abs(pos), 0.0, 1.0); 
      
      radius = estimate_turn_radius(pos, 1.0/r_max);
      radius = 5.0*sgn(pos)*clip(radius, r_min, r_max);    

      float new_speed = (1.0 - ws)*speed_max + ws*speed_min;
      speed = clip(speed + ramp, speed_min, new_speed);
    
    } 
    else if (line_sensor.line_lost_type == LINE_LOST_LEFT) 
    {
      radius = r_min;  
      speed  = 1.5*speed_min; 
    }
    else if (line_sensor.line_lost_type == LINE_LOST_RIGHT)
    {
      radius = -r_min;
      speed  = 1.5*speed_min;
    }
    else if (line_sensor.line_lost_type == LINE_LOST_CENTER) 
    {
      return; 
    } 

    position_control.set_circle_motion(radius, speed);
    timer.delay_ms(4);
  }
} 



     



void line_followingB(PositionControlLQR &position_control, float r_min, float r_max, float speed_min, float speed_max)
{
  Gpio<LED_1_GPIO, LED_1_PIN, GPIO_MODE_OUT> led_1;   //user led
    

  uint32_t steps = 0;

  float speed_min_curr = 0.0;

  FirFilter<float, 64> quality_filter(1.0);

  while (1)       
  {
    

    position_control.enable_lf();
    //position_control.disable_lf();

    speed_min_curr = clip(speed_min_curr + speed_min/100.0, 0.0, speed_min);

    float position = line_sensor.left_position; 

    quality_filter.step(abs(position));

    float radius  = estimate_turn_radius(position, 1.0/r_max);
    radius  = -sgn(position)*clip(radius, r_min, r_max);      

    float q = 1.0 - 1.5*quality_filter.max(); 

    q = clip(q, 0.0, 1.0);         

    //if quality is high (close to 1), increase radius - allows faster speed
    float kr = q*6.0 + (1.0 - q)*2.0;  

    //if quality is high (close to 1), use higher speed
    float speed = q*speed_max + (1.0 - q)*speed_min_curr;  

    position_control.set_circle_motion(kr*radius, speed);
   
    if (line_sensor.line_lost_type == LINE_LOST_CENTER)   
    {
      break; 
    }  

    timer.delay_ms(4);

    steps++;
  }

  led_1 = 0;
  position_control.disable_lf();
  position_control.set(position_control.distance, position_control.angle); 
} 
*/