#ifndef _SHARP_TURN_DETECT_H_
#define _SHARP_TURN_DETECT_H_

class SharpTurnDetect
{
    public:
        void init(float max_distance, float threshold);
        bool step(float position);

    private:
        float max_distance,  threshold;
        int sharp_turn_state;
        int sharp_turn_mark;

};


#endif