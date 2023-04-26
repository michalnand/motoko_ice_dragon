import numpy
#from render import *

'''
state : x, v, theta, w

dv     = (-1.0/tau1)*v + (1.0/tau1)*b1*(ur + ur)/2
dw     = (-1.0/tau2)*v + (1.0/tau2)*b2*(ur - ul)
dx     = v
dtheta = w
'''
class RobotModel:
    def __init__(self, dt = 0.001):

        self.dt = dt

        
        tau1 = 0.3
        tau2 = 0.5
        b1   = 2.7
        b2   = 0.7

      

        self.mat_a = numpy.zeros((4, 4))
        self.mat_b = numpy.zeros((4, 2))
        self.mat_c = numpy.zeros((4, 4)) 


        self.mat_a[0][0] = -1.0/tau1
        self.mat_a[1][1] = -1.0/tau2
        self.mat_a[2][0] = 1.0
        self.mat_a[3][1] = 1.0


        self.mat_b[0][0] = (1.0/tau1)*b1/2.0
        self.mat_b[0][1] = (1.0/tau1)*b1/2.0

        self.mat_b[1][0] = (1.0/tau2)*b2
        self.mat_b[1][1] = (-1.0/tau2)*b2 
    

        
        self.mat_c[0][0] = 1.0
        self.mat_c[1][1] = 1.0
        self.mat_c[2][2] = 1.0
        self.mat_c[3][3] = 1.0
        

        #self.renderer = Render(512, 512)


    def forward(self, x, u):
        dx = self.mat_a@x + self.mat_b@u

        x  = x + dx*self.dt
        y  = self.mat_c@x
        
        return x, y

    
    def render(self, y):
        x_pos = y[0][0]
        y_pos = 0
        theta = y[1][0]*180.0/numpy.pi
        phi   = 0.0

        
        
        #self.renderer.render(x_pos, y_pos, phi, theta)