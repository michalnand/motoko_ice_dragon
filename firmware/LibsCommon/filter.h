#ifndef _FILTER_H_
#define _FILTER_H_



template<class DType, int size>
class FirFilter
{
    public:
        FirFilter()
        {
            ptr = 0;
            for (unsigned int i = 0; i < size; i++)
            {
                this->x[i] = 0;
            }

            result = 0;
        }

        DType step(DType x)
        {
            result-= this->x[ptr];
            this->x[ptr] = x;
            result+= this->x[ptr];

            ptr = (ptr + 1)%size;

            return result/size;
        }

    private:
        unsigned int ptr;
        DType x[size];
        DType result;
};




template<class DType, int size, DType threshold>
class OutlierFirFilter
{
    public:
        OutlierFirFilter()
        {
            ptr = 0;
            for (unsigned int i = 0; i < size; i++)
            {
                this->x[i] = 0;
            }
        }

        DType step(DType x)
        {
            this->x[ptr] = x;
            ptr = (ptr + 1)%size;


            DType mean = 0;
            for (unsigned int i = 0; i < size; i++)
            {
                mean+= this->x[i];
            }
            mean = mean/size;


            DType result = 0;
            for (unsigned int i = 0; i < size; i++)
            {
                auto d = this->x[i] - mean;

                if (d < 0)
                {
                    d = -d;
                }

                if (d > threshold)
                {
                    result+= mean;
                }
                else 
                {
                    result+= this->x[i];
                }
            }

            result = result/size;


            return result;
        }

    private:
        unsigned int ptr;
        DType x[size];
};


#endif