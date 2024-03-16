#ifndef _SHARP_DETECT_H_
#define _SHARP_DETECT_H_

class SharpRightDetect
{

    public:
        void init(float angle_turn = 60.0, float angle_threshold = 0.8);
        bool step();

    private:
        bool  turn_mark;
        float turn_angle;
        float end_angle;
        float angle_threshold;
};





#endif