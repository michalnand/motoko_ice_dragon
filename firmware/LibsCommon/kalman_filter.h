#ifndef  _KalmanFilter_H_
#define  _KalmanFilter_H_ 

class KalmanFilter
{
    public:
        void init();

        /*
            z   - position measurement
            pz  - position measurement uncertainity (variance)
        */
        float step(float z, float pz);

    private:
        float x_hat;
        float p;
};
 

#endif
