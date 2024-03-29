#include "line_following.h"
#include <fmath.h>

//#include <position_control_lqr.h>
#include <position_control_lqg.h>

#include <trajectory_tracking.h>

  
LineFollowing::LineFollowing()
{
    this->r_min = 80.0;
    this->r_max = 10000.0;

    this->speed_min = 150.0;
    this->speed_max = 300.0; //150.0, 250.0, 300.0, 400
    //this->speed_max = 150.0; //150.0, 250.0, 300.0, 400


    this->q_penalty = 1.0;   
    this->qr_max    = 8.0;      
    this->qr_min    = 2.0;  


    sharp_turn_detect.init(60.0, 0.8);
}




int LineFollowing::main()
{
    this->line_search(LINE_LOST_CENTER);

    float speed_min_curr = 0.0;

    quality_filter.init(1.0);

    while (1)
    {
        float position = line_sensor.right_position; 

        if (line_sensor.line_lost_type != LINE_LOST_NONE)   
        {
          position_control.disable_lf();
          this->line_search(line_sensor.line_lost_type);
        } 
        

        position_control.enable_lf();

        speed_min_curr = clip(speed_min_curr + speed_min/10.0, 0.0, speed_min);

        quality_filter.step(abs(position));

        float radius  = this->estimate_turn_radius(position, 1.0/r_max);
        radius  = -sgn(position)*clip(radius, r_min, r_max);      

        float q = 1.0 - this->q_penalty*quality_filter.max(); 
      
        q = clip(q, 0.0, 1.0);          

        //if quality is high (close to 1), increase radius - allows faster speed
        float kr = q*this->qr_max + (1.0 - q)*this->qr_min;  

        //if quality is high (close to 1), use higher speed
        float speed = q*speed_max + (1.0 - q)*speed_min_curr;  

        position_control.set_circle_motion(kr*radius, speed);

        timer.delay_ms(4);
    }   

    position_control.disable_lf();
    position_control.set(position_control.distance, position_control.angle);

    return 0;
}


void LineFollowing::line_search(uint32_t line_lost_type)
{
  TrajectoryTracking trajectory_tracking;

  position_control.disable_lf();

  float search_distance = 100.0;
  float search_angle    = 130.0*PI/180.0;

  int      way = 1;
  uint32_t state = 0.0;

  if (line_lost_type == LINE_LOST_LEFT)
  {
    way   = 1;
    state = 0;
  }
  else if (line_lost_type == LINE_LOST_RIGHT)
  {
    way   = -1;
    state = 0;
  }
  else
  {
    way = 1;
    state = 1.0;
  }

  while (true)
  {
    //side line search
    if (state == 0 || state == 1)
    {
      trajectory_tracking.start(search_distance, way*search_angle);
      while (trajectory_tracking.step() != true)
      { 
        timer.delay_ms(4);  

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      } 

      timer.delay_ms(50);      

      trajectory_tracking.start(-search_distance, -way*search_angle);
      while (trajectory_tracking.step() != true)
      {
        timer.delay_ms(4); 

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      } 

      timer.delay_ms(50);

      way*= -1;
      state++;
    }

    //forward line search
    else
    {
      trajectory_tracking.start(search_distance, 0.0);
      while (trajectory_tracking.step() != true)
      {
        timer.delay_ms(4); 

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      } 

      state = 0;
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








