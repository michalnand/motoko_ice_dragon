#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

#include <drivers.h>
#include <filter.h>
#include <sharp_detect.h>

class LineFollowing
{
    public: 
        LineFollowing();
        
        int main();

    private:
        float estimate_turn_radius(float sensor_reading, float eps);
        void line_search(uint32_t line_lost_type);

        
    private:
        FirFilter<float, 64> quality_filter;
        SharpRightDetect sharp_turn_detect;
        
        float r_min, r_max, speed_min, speed_max;

        float q_penalty, qr_max, qr_min;
};
 

#endif