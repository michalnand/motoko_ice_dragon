#ifndef _MATRIX_H_
#define _MATRIX_H_


template<class DType, unsigned int N, unsigned int M> 
class Matrix
{
    public:
        Matrix()
        {
            
        }

        Matrix(const DType &init_value)
        {
            for (unsigned int i = 0; i < N*M; i++)
            {
                x[i] = init_value;
            }
        } 

        Matrix(const Matrix<DType, N, M> &rhs)
        {
            for (unsigned int i = 0; i < N*M; i++)
                x[i] = rhs.x[i];
        }

        virtual ~Matrix()
        {

        }

    public:
        Matrix<DType, N, M> operator =(const Matrix<DType, N, M> &rhs)
        {
            for (unsigned int i = 0; i < N*M; i++)
            {
                x[i] = rhs.x[i];
            }

            return *this;
        }   

        DType& operator [](unsigned int idx)
        {
            return x[idx];
        }

    public:
        Matrix<DType, N, M> operator +(Matrix<DType, N, M> rhs)
        {
            Matrix<DType, N, M> result;

            for (unsigned int i = 0; i < N*M; i++)
            {
                result[i] = x[i] + rhs[i];
            }

            return result;
        }

        Matrix<DType, N, M> operator -(Matrix<DType, N, M> rhs)
        {
            Matrix<DType, N, M> result;

            for (unsigned int i = 0; i < N*M; i++)
            {
                result[i] = x[i] - rhs[i];
            }

            return result;
        }

        Matrix<DType, N, M> operator *(DType v)
        {
            Matrix<DType, N, M> result;

            for (unsigned int i = 0; i < N*M; i++)
            {
                result[i] = x[i]*v;
            }

            return result;
        }


        template<unsigned int K>
        Matrix<DType, N, K> operator *( Matrix<DType, M, K> &rhs)
        {
            Matrix<DType, N, K> result;

            for (unsigned int i = 0; i < N; i++)
            {
                for (unsigned int j = 0; j < K; j++)
                {
                    DType sum = 0;
                    for (unsigned int k = 0; k < M; k++)
                    {
                        sum+= x[i*M + k]*rhs[k*K + j]; 
                    }  

                    result.set(i*K + j, sum);
                }
            }

            return result;
        }

    public:
        void init(DType value = 0)
        {
            for (unsigned int i = 0; i < N*M; i++)
            {
                x[i] = value;
            }
        }

        void set(unsigned int idx, DType value)
        {
            x[idx] = value;
        }

        DType get(unsigned int idx)
        {
            return x[idx]; 
        }
 
    private:
        DType x[N*M];
};


#endif