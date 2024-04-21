#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

#include <drivers.h>
#include <filter.h>
#include <split_line_detector.h>

class LineFollowing
{
    public: 
        LineFollowing();
        
        int main();

    public:
        
        void line_search(uint32_t line_lost_type);
        void obstacle_avoid();
        void curtain_avoid();
        
    public:
        float estimate_turn_radius(float sensor_reading, float eps);



        
    private:
        FirFilter<float, 32> quality_filter;
        SplitLineDetector split_line_detector;
        
        float r_min, r_max, speed_min, speed_max;

        float q_penalty, qr_max, qr_min;

        int obstacle_idx;
        Array<bool, 4> obstacle_map;
};  
 

#endif