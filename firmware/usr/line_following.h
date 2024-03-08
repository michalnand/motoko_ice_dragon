#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

#include <drivers.h>
#include <filter.h>


class LineFollowing
{
    public: 
        LineFollowing();
        
        int main();

    private:
        float estimate_turn_radius(float sensor_reading, float eps);
        void line_search(uint32_t line_lost_type);

        bool double_line_detect(float max_distance);
        
    private:
        FirFilter<float, 64> quality_filter;
        FirFilter<float, 32> left_positon_filter;
        FirFilter<float, 32> right_positon_filter;

        float r_min, r_max, speed_min, speed_max;

        float q_penalty, qr_max, qr_min;

        float left_turn_distance, right_turn_distance;
};
 

#endif