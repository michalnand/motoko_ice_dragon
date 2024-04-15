#ifndef _MATRIX_H_
#define _MATRIX_H_


// M - num of rows
// N - num of cols
template<class DType, unsigned int M, unsigned int N> 
class Matrix
{
    public:
        Matrix()
        {
            
        }

        Matrix(const DType &init_value)
        {
            for (unsigned int i = 0; i < M*N; i++)
            {
                x[i] = init_value;
            }
        } 

        Matrix(const Matrix<DType, M, N> &rhs)
        {
            for (unsigned int i = 0; i < M*N; i++)
            {
                x[i] = rhs.x[i];
            } 
        } 

        virtual ~Matrix()
        {

        }

    public:
        Matrix<DType, M, N> operator =(const Matrix<DType, M, N> &rhs)
        {
            for (unsigned int i = 0; i < M*N; i++)
            {
                x[i] = rhs.x[i];
            }

            return *this;
        }   

        DType& operator [](unsigned int idx)
        {
            return x[idx];
        }

    //common operators
    public:
        //two matrix addition
        Matrix<DType, M, N> operator +(Matrix<DType, M, N> rhs)
        {
            Matrix<DType, M, N> result; 

            for (unsigned int i = 0; i < M*N; i++)
            {
                result[i] = x[i] + rhs[i];
            }

            return result;
        }

        //two matrix substraction
        Matrix<DType, M, N> operator -(Matrix<DType, M, N> rhs)
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                result[i] = x[i] - rhs[i];
            }

            return result;
        }


        //add scalar from matrix
        Matrix<DType, M, N> operator +(DType v)
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                result[i] = x[i] + v;
            }

            return result;
        }

        //substract scalar from matrix
        Matrix<DType, M, N> operator -(DType v)
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                result[i] = x[i] - v;
            }

            return result;
        }

        //multiply matrix by scalar
        Matrix<DType, M, N> operator *(DType v)
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                result[i] = x[i]*v;
            }

            return result;
        }

        /*
            matmul 

            mat_y.shape(M, K)
            mat_a.shape(M, N) (this)
            mat_b.shape(N, K) (rhs)

            y = a*b
        */
        template<unsigned int K>
        Matrix<DType, M, K> operator *( Matrix<DType, N, K> &rhs)
        {
            Matrix<DType, M, K> result; 

            for (unsigned int m = 0; m < M; m++)
            {
                for (unsigned int k = 0; k < K; k++)
                {
                    DType sum = 0; 

                    for (unsigned int n = 0; n < N; n++)
                    {
                        sum+= x[m*N + n]*rhs[n*K + k];
                    }  

                    result[m*K + k] = sum;
                }
            }

            return result;
        }


        DType sum()
        {
            DType result = 0;

            for (unsigned int i = 0; i < M*N; i++)
            {
                result+=x[i];
            }

            return result;
        }

        Matrix<DType, 1, N> sum_cols()
        {
             Matrix<DType, 1, N> result; 

            for (unsigned int j = 0; j < N; j++)
            {
                DType sum = 0;
                for (unsigned int i = 0; i < M; i++)
                {
                    sum+= x[i*N + j];
                }
                
                result[j] = sum;
            }

            return result;
        }


        Matrix<DType, M, 1> sum_rows()
        {
            Matrix<DType, M, 1> result;

            for (unsigned int j = 0; j < M; j++)
            {
                DType sum = 0;
                for (unsigned int i = 0; i < N; i++)
                {
                    sum+= x[i*N + i];
                }
                
                result[j] = sum;
            }

            return result;
        }

    public:
        void init(DType value = 0)
        {
            for (unsigned int i = 0; i < M*N; i++)
            {
                x[i] = value;
            }
        }


        void from_array(DType *v)
        {
            for (unsigned int i = 0; i < M*N; i++)
            {
                x[i] = v[i];
            }
        }


        void diag()
        {
            unsigned int count;
            if (M < N)
            {
                count = M;
            }
            else
            {
                count = N;
            }

            for (unsigned int i = 0; i < count; i++)
            {
                x[i + i*N] = 1;
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

        unsigned int rows()
        {
            return M;
        }

        unsigned int cols()
        {
            return N;
        }

        unsigned int size()
        {
            return M*N;
        }

    //nonlinear operations
    public:
        Matrix<DType, M, N> clip(DType min_v, DType max_v)
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                auto tmp = x[i]; 

                if (tmp < min_v)
                {
                    tmp = min_v;
                }
                else if (tmp > max_v)
                {
                    tmp = max_v;
                }
 
                result[i] = tmp;
            }

            return result;
        }


        Matrix<DType, M, N> shrink(DType min_v, DType max_v)
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                auto tmp = x[i];

                if (tmp < 0 && tmp > min_v)
                {
                    tmp = 0;
                }

                if (tmp > 0 && tmp < max_v)
                {
                    tmp = 0;
                }
 
                result[i] = tmp;
            }

            return result;
        }


        Matrix<DType, M, N> sign()
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                auto tmp = x[i];

                if (tmp < 0)
                {
                    tmp = -1;
                }
                else if (tmp > 0)
                {
                    tmp = -1;
                }
                else
                {
                    tmp = 0;
                }
 
                result[i] = tmp;
            }

            return result;
        }


        Matrix<DType, M, N> abs()
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                auto tmp = x[i];

                if (tmp < 0)
                {
                    tmp = -tmp;
                }
              
                result[i] = tmp;
            }

            return result;
        }

        Matrix<DType, M, N> sqr()
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < M*N; i++)
            {
                auto tmp = x[i];

                result[i] = tmp*tmp;
            }

            return result;
        }
 
    public:
        Matrix<DType, M, N> transpose()
        {
            Matrix<DType, M, N> result;

            for (unsigned int i = 0; i < N; i++)
            {
                for (unsigned int j = 0; j < M; j++)
                {
                    result[j*N + i] = x[M*i + j];
                }
            }

            return result;
        }

        template<unsigned int N2> 
        Matrix<DType, M, N + N2> hstack( Matrix<DType, M, N2> &rhs)
        {
            Matrix<DType, M, N + N2> result;

            for (unsigned int m = 0; m < M; m++)
            {
                unsigned int col_ptr = 0;

                for (unsigned int n = 0; n < N; n++)
                {
                    result[m*(N + N2) + col_ptr] = x[m*N + n];
                    col_ptr++;
                }

                for (unsigned int n = 0; n < N2; n++)
                {
                    result[m*(N + N2) + col_ptr] = rhs[m*N2 + n];
                    col_ptr++;
                } 
            }

            return result;
        }

        template<unsigned int M2>
        Matrix<DType, M + M2, N> vstack( Matrix<DType, M + M2, N> &rhs)
        {
            Matrix<DType, M + M2, N> result;
            
            for (unsigned int i = 0; i < M*N; i++)
            {
                result[i] = x[i];
            }

            for (unsigned int i = 0; i < M2*N; i++)
            {
                result[i + M*N] = x[i];
            } 
           
            return result;
        }



        Matrix<DType, 1, N> min_cols()
        {
            Matrix<DType, 1, N> result(0);

            for (unsigned int n = 0; n < N; n++) 
            {
                DType v = x[n]; 

                for (unsigned int m = 0; m < M; m++)
                {
                    DType tmp = x[m*N + n];
                    if (tmp < v)
                    {
                        v = tmp;
                    }
                } 

                result[n] = v;
            }
            
            return result;
        }

        Matrix<DType, 1, N> max_cols()
        { 
            Matrix<DType, 1, N> result(0);

            for (unsigned int n = 0; n < N; n++) 
            {
                DType v = x[n]; 

                for (unsigned int m = 0; m < M; m++)
                {
                    DType tmp = x[m*N + n];
                    if (tmp > v)
                    {
                        v = tmp;
                    }
                } 

                result[n] = v;
            }
            
            return result;
        }


        bool all_in_range(DType min_value, DType max_value)
        { 
            for (unsigned int i = 0; i < N*M; i++) 
            {
                if (x[i] < min_value)
                {
                    return false;
                }

                if (x[i] > max_value)
                {
                    return false;
                }
            }

            return true;
        }

    private:
        DType x[M*N];
};


#endif