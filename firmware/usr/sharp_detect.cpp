#include "sharp_detect.h"

#include "fmath.h"
#include "drivers.h"
#include "position_control_lqg.h"

void SharpRightDetect::init(float turn_angle, float angle_threshold)
{
    this->turn_mark     = false;
    this->end_angle     = 0.0;
    this->turn_angle        = turn_angle*(PI/180.0);
    this->angle_threshold   = angle_threshold*(PI/180.0);
}

bool SharpRightDetect::step()
{
    float position = line_sensor.right_position;

    if (turn_mark == false && position > angle_threshold) 
    {
        turn_mark        = true;
        end_angle        = -(position_control.angle + turn_angle);
    }
    else if (turn_mark == true)
    {
      float angle_dif = end_angle - position_control.angle; 

      //terminal << end_angle << " " << position_control.angle << " " << angle_dif << "\n";

      if (angle_dif > 0.0 && position < (0.5*angle_threshold))
      {
        turn_mark = false;
      }  
    }

    return turn_mark;
}  

