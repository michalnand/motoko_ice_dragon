import numpy

import matplotlib.pyplot as plt

MOTOR_CONTROL_MAX = 1024
SINE_TABLE_MAX    = 512
SINE_TABLE_SIZE   = 1024
ENCODER_RESOLUTION= 4096
MOTOR_POLES       = 14


def set_phases(torque, rotor_angle, sine_table):


    electrical_angle = rotor_angle*MOTOR_POLES*SINE_TABLE_SIZE/(2*ENCODER_RESOLUTION)
    electrical_angle = int(electrical_angle)



    a = torque*sine_table[(electrical_angle + 0*SINE_TABLE_SIZE//3)%SINE_TABLE_SIZE]/SINE_TABLE_MAX
    b = torque*sine_table[(electrical_angle + 1*SINE_TABLE_SIZE//3)%SINE_TABLE_SIZE]/SINE_TABLE_MAX
    c = torque*sine_table[(electrical_angle + 2*SINE_TABLE_SIZE//3)%SINE_TABLE_SIZE]/SINE_TABLE_MAX


   
    #transform into space-vector modulation, to achieve full voltage range
    min_val = min([a, b, c])
    max_val = max([a, b, c])

    com_val = (min_val + max_val)/2.0

    '''
    #normalise into 0..MOTOR_CONTROL_MAX
    a_norm = ( (a - com_val)/numpy.sqrt(3) + MOTOR_CONTROL_MAX/2 )
    b_norm = ( (b - com_val)/numpy.sqrt(3) + MOTOR_CONTROL_MAX/2 )
    c_norm = ( (c - com_val)/numpy.sqrt(3) + MOTOR_CONTROL_MAX/2 )

    '''
    a_norm = (a + SINE_TABLE_MAX)/2
    b_norm = (b + SINE_TABLE_MAX)/2
    c_norm = (c + SINE_TABLE_MAX)/2

    return a, b, c, a_norm, b_norm, c_norm


if __name__ == "__main__":

    x = 2.0*numpy.pi*numpy.arange(0, SINE_TABLE_SIZE)/SINE_TABLE_SIZE
    sine_table = SINE_TABLE_MAX*numpy.sin(x)    


    theta_res = []

    a_res      = []
    b_res      = []
    c_res      = []

    a_norm_res      = []
    b_norm_res      = []
    c_norm_res      = []

    torque = 500

    for rotor_angle in range(ENCODER_RESOLUTION):
        theta_res.append((rotor_angle/ENCODER_RESOLUTION)*360.0)

        a, b, c, a_norm, b_norm, c_norm = set_phases(torque, rotor_angle, sine_table)


        a_res.append(a)
        b_res.append(b)
        c_res.append(c)

        a_norm_res.append(a_norm)
        b_norm_res.append(b_norm)
        c_norm_res.append(c_norm)

    

    plt.clf()
    fig, axs = plt.subplots(3, 1, figsize=(8, 8))


    axs[0].plot(theta_res, label="theta", color="deepskyblue")
    axs[0].set_xlabel("rotor step")
    axs[0].set_ylabel("angle [degrees]")
    axs[0].grid()


    axs[1].plot(a_res, label="a", color="red")
    axs[1].plot(b_res, label="b", color="green")
    axs[1].plot(c_res, label="c", color="blue")
    axs[1].set_xlabel("step")
    axs[1].set_ylabel("current")
    axs[1].legend()
    axs[1].grid()


    axs[2].plot(a_norm_res, label="PWM a", color="red")
    axs[2].plot(b_norm_res, label="PWM b", color="green")
    axs[2].plot(c_norm_res, label="PWM c", color="blue")
    axs[2].set_xlabel("step")
    axs[2].set_ylabel("pwm normalised")
    axs[2].legend()
    axs[2].grid()
        
    plt.tight_layout()
    plt.show()
    #plt.savefig("output.png", dpi = 300)