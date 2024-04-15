#ifndef _FILTER_H_
#define _FILTER_H_



template<class DType, int size>
class FirFilter
{
    public:
        FirFilter(DType value = 0)
        {
            this->init(value);
        }

        void init(DType value)
        {
            ptr = 0;
            for (unsigned int i = 0; i < size; i++)
            {
                this->x[i] = value;
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

        DType max() 
        {
            DType result = this->x[0];

            for (unsigned int i = 0; i < size; i++)
                if (this->x[i] > result)
                {
                    result = this->x[i];
                }

            return result;
        }   

        DType min() 
        {
            DType result = this->x[0];

            for (unsigned int i = 0; i < size; i++)
                if (this->x[i] < result)
                {
                    result = this->x[i];
                }

            return result;
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