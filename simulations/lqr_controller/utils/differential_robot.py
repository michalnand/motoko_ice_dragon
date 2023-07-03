import numpy
from .render import *


class DifferentialRobot:
    def __init__(self, dt = 0.001):

        self.dt = dt

        #time constant for yaw angle robot rotation (steering)
        tau_turn    = 1.0/0.3

        #time constant for robot acceleration (forward direction)
        tau_forward = 1.0/0.5

        #amplification, ratio between measured value amplitude : controll variable amplitude
        b_turn      = 1.5
        b_forward   = 0.2



        self.mat_a = numpy.zeros((4, 4))
        self.mat_b = numpy.zeros((4, 2))
        self.mat_c = numpy.zeros((4, 4)) 


        self.mat_a[0][0] =  -9.831
        self.mat_a[0][1] =  -1.397
        self.mat_a[1][0] =  1.0
        self.mat_a[2][2] =  -tau_forward
        self.mat_a[3][2] =  1.0


        self.mat_b[0][0] =   98.806
        self.mat_b[0][1] =   -98.806
        
        self.mat_b[2][0] =  b_forward*tau_forward
        self.mat_b[2][1] =  b_forward*tau_forward

       
        self.mat_c[0][0] = 1.0
        self.mat_c[1][1] = 1.0
        self.mat_c[2][2] = 1.0
        self.mat_c[3][3] = 1.0
        

        #state
        self.x       = numpy.zeros((self.mat_a.shape[0], 1))
        self.y       = numpy.zeros((self.mat_c.shape[0], 1))

        #cartesian state space
        self.theta = 0
        self.x_pos = 0
        self.y_pos = 0

    def copy(self):
        result = DifferentialRobot()

        result.theta = self.theta
        result.x_pos = self.x_pos
        result.y_pos = self.y_pos

        return result
    

    def reset(self):
        #state
        self.x       = numpy.zeros((self.mat_a.shape[0], 1))
        self.y       = numpy.zeros((self.mat_c.shape[0], 1))

        #cartesian state space
        self.theta = 0
        self.x_pos = 0
        self.y_pos = 0

    def forward(self, u):
        dx = self.mat_a@self.x + self.mat_b@u

        self.x  = self.x + dx*self.dt
        self.y  = self.mat_c@self.x

        #angular velocity
        w  = self.y[0, 0]
        #velocity
        v = self.y[2, 0]


        theta = self.y[1, 0]

        #position in cartesian
        self.x_pos+= v*numpy.cos(theta)*self.dt
        self.y_pos+= v*numpy.sin(theta)*self.dt
        self.theta+= w*self.dt
         
        #wrap into 0 .. 2pi
        self.theta = numpy.mod(self.theta, 2.0*numpy.pi)

        return self.x, self.y

   