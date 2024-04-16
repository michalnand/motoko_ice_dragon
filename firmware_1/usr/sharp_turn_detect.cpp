#include "sharp_turn_detect.h"

#include "fmath.h"
#include "drivers.h"
#include "position_control_lqg.h"


void SharpTurnDetect::init(float max_distance, float threshold)
{
  this->max_distance = max_distance;
  this->threshold    = threshold;

  sharp_turn_state  = 0;
  sharp_turn_mark   = 0;
}
      
bool SharpTurnDetect::step(float position)
{
  if (sharp_turn_state == 0)
  {
    if (position > threshold)
    {
      sharp_turn_mark = position_control.distance + max_distance;
      sharp_turn_state = 1;
      return false;
    }
  }

  if (sharp_turn_state == 1)
  {
    if (position_control.distance > sharp_turn_mark)
    {
      sharp_turn_state = 0;
      return false;
    }
    else if (position < -threshold)
    {
      sharp_turn_state = 0;
      return true;
    }
  }

  return false;
}
