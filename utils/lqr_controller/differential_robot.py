import numpy
from render import *


class DifferentialRobot:
    def __init__(self, dt = 0.001):

        self.dt = dt

        #time constant for yaw angle robot rotation (steering)
        tau_turn    = 6.3

        #time constant for robot acceleration (forward direction)
        tau_forward = 0.9

        #amplification, ratio between measured value amplitude : controll variable amplitude
        b_turn      = 1.5
        b_forward   = 0.2



        self.mat_a = numpy.zeros((4, 4))
        self.mat_b = numpy.zeros((4, 2))
        self.mat_c = numpy.zeros((4, 4)) 


        self.mat_a[0][0] =  -tau_turn
        self.mat_a[1][0] =  1.0
        self.mat_a[2][2] = -tau_forward
        self.mat_a[3][2] = 1.0


        self.mat_b[0][0] =  -b_turn*tau_turn
        self.mat_b[0][1] =   b_turn*tau_turn
        
        self.mat_b[2][0] =  b_forward*tau_forward
        self.mat_b[2][1] =  b_forward*tau_forward

       
        self.mat_c[0][0] = 1.0
        self.mat_c[1][1] = 1.0
        self.mat_c[2][2] = 1.0
        self.mat_c[3][3] = 1.0
        

        self.renderer = Render(700, 700)

        #cartesian state space
        self.theta = 0
        self.x_pos = 0
        self.y_pos = 0


    def forward(self, x, u):
        dx = self.mat_a@x + self.mat_b@u

        x  = x + dx*self.dt
        y  = self.mat_c@x

        #angular velocity
        w  = y[0, 0]
        #velocity
        v = y[2, 0]


        #position in cartesian
        self.x_pos+= v*numpy.cos(self.theta)*self.dt
        self.y_pos+= v*numpy.sin(self.theta)*self.dt
        self.theta+= w*self.dt
         
        
        #wrap into -pi .. pi
        #self.theta = numpy.arctan2(numpy.sin(self.theta), numpy.cos(self.theta))

        #wrao into 0 .. 2pi
        self.theta = numpy.mod(self.theta, 2.0*numpy.pi)


        return x, y

    
    def render(self, target_x_pos, target_y_pos):
        
        
        self.renderer.render(self.x_pos, self.y_pos, self.theta, target_x_pos, target_y_pos)