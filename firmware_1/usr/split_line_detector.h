#ifndef _SPLIT_LINE_DETECTOR_H_
#define _SPLIT_LINE_DETECTOR_H_


class SplitLineDetector
{
    public:
        void init(float max_distance, float threshold);
        void reset();
        int step(float left_position, float right_position);

    private:
        float   max_distance,  threshold;
        int     state;  
        float   distance_mark;
        float   left_position, right_position;
};

#endif  