#include "split_line_detector.h"

#include "fmath.h"
#include "drivers.h"
#include "position_control_lqg.h" 


void SplitLineDetector::init(float max_distance, float threshold)
{
  this->max_distance = max_distance;
  this->threshold    = threshold;

  state           = 0;
  distance_mark   = 0;
  left_position   = 0;
  right_position  = 0;
}

void SplitLineDetector::reset()
{
  state       = 0;
}

int SplitLineDetector::step(float left_position, float right_position)
{
  if (state == 0) 
  {
    if (right_position > threshold)
    {
      distance_mark        = position_control.distance + max_distance;
      this->left_position  = left_position;
      this->right_position = right_position;

      state                = 1;
    }
  }

  else if (state == 1)
  {
    if (position_control.distance > distance_mark)
    {
      state = 0;
    }
    else  
    {
      if (left_position < -threshold)
      {
        distance_mark  = position_control.distance + max_distance;
        state = 2;
        
        return 1;
      }
    }
  }
  else if (state == 2)
  {
    if (position_control.distance > distance_mark)
    {
      state = 0;
    }
  }

  return 0;
}


/*
int SplitLineDetector::step(float position)
{
  if (state == 0) 
  {
    if (position > threshold)
    {
      distance_mark = position_control.distance + max_distance;
      angle_mark    = position_control.angle;
      state = 1;

      return 0;
    }
  }

  if (state == 1)
  {
    if (position_control.distance > distance_mark)
    {
      state = 0;
      return 0;
    }
    else if (position < -threshold)
    {
      state = 0;

      if (position_control.angle > angle_mark)
      {
        return 1;
      }
      else
      {
        return -1;
      }
    }
  }

  return 0;
}
*/
