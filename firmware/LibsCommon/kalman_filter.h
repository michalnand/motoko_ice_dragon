#ifndef  _KalmanFilter_H_
#define  _KalmanFilter_H_ 

class KalmanFilter
{
    public:
        KalmanFilter();

        void init();

        /*
            z   - position measurement
            pz  - position measurement uncertaininty (variance)
        */
        float step(float z, float pz);

    private:
        float x_hat;
        float p;
};


#endif
