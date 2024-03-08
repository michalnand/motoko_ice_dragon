
import numpy



def create_data(n_steps, k_forward, k_turn, tau_forward, tau_turn):

    mat_a = numpy.zeros((4, 4), dtype=numpy.float32)
    mat_b = numpy.zeros((4, 2), dtype=numpy.float32)

    mat_a[0][0] = 1.0
    mat_a[0][2] = 1.0
    mat_a[1][1] = 1.0
    mat_a[1][3] = 1.0

    mat_a[2][2] = tau_forward
    mat_a[3][3] = tau_turn

    mat_b[2][0] = k_forward
    mat_b[2][1] = k_forward

    mat_b[3][0] = k_turn
    mat_b[3][1] = -k_turn

    print("mat_a = ")
    print(mat_a)
    print("\n\n")
    print("mat_b = ")
    print(mat_b)
    print("\n\n")

    u_result = numpy.zeros((n_steps, 2), dtype=numpy.float32)
    x_result = numpy.zeros((n_steps, 4), dtype=numpy.float32)

    u = numpy.zeros((2, 1), dtype=numpy.float32)
    x = numpy.zeros((4, 1), dtype=numpy.float32)

    #u_left  = [0.0,  -1.0, 1.0,  1.0, -1.0]
    #u_right = [0.0,   1.0, -1.0, 1.0, -1.0]

    u_left  = [0.0,  1.0, -1.0, 0.0,  0.0, 1.0, -1.0]
    u_right = [0.0,  0.0,  0.0, 1.0, -1.0, -1.0, 1.0]

    for n in range(n_steps):
        idx = (n//50)%len(u_left); 

        u[0][0] = u_left[idx] 
        u[1][0] = u_right[idx]

        u_result[n] = u[:, 0]
        x_result[n] = x[:, 0]

        x_new = mat_a@x + mat_b@u
        x = x_new.copy()

    return u_result, x_result

