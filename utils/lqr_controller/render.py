import numpy

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import glfw




class Render:
    def __init__(self, height, width):        

        self.width = width
        self.height = height

        res = self._create_window(self.height, self.width)
        print("init res = ", res)

        self.path = []
        
    def _create_window(self, height, width, label = "visualisation"):

        # Initialize the library
        if not glfw.init():
            return -1
        
        # Create a windowed mode window and its OpenGL context
        self.window = glfw.create_window(width, height, label, None, None)
        if not self.window:
            glfw.terminate()
            return -2

        
        # Make the window's context current
        glfw.make_context_current(self.window)

        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(100.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(self.width)/float(self.height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
        return 0


    def render(self, x_pos, y_pos, theta, target_x_pos, target_y_pos):

        self.path.append([x_pos, y_pos])

        if len(self.path) > 100:
            self.path = self.path[1:]

        '''
        #glViewport(-1, -1, 2*self.width, 2*self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        glEnable(GL_DEPTH_TEST)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (self.width/self.height), 0.1, 50.0)

        glMatrixMode(GL_MODELVIEW)
        '''

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) 
        glLoadIdentity() 

       
        glClearColor(0.18, 0.09, 0.2, 0)


        glTranslatef(0.0, 0.0, -4.0)

        self.rotate(50, 0, 0)


        glPushMatrix()
        
        grid_size = 20
        d = 2/(grid_size)

        for j in range(grid_size):
            for i in range(grid_size):

                glPushMatrix()

                if (j%2) == (i%2):
                    glColor3f(0.3, 0.3, 0.3)
                else:
                    glColor3f(0.1, 0.1, 0.1)
                    

                x = 4.0*(j/grid_size - 0.5)
                y = 4.0*(i/grid_size - 0.5)

                glTranslatef(x, 0, y)

                glBegin(GL_QUADS)
                glVertex3f(d, 0, -d)
                glVertex3f(-d, 0, -d)
                glVertex3f(-d, 0, d)
                glVertex3f(d, 0, d)
                glEnd()

                glPopMatrix()
        

        glPopMatrix()


        glPushMatrix()

        glColor3f(1.0, 1.0, 1.0)
        glTranslatef(target_x_pos, 0.0, -target_y_pos)
        self.paint_cuboid(0.1, 0.1, 0.1)
        glPopMatrix()

        glPushMatrix()

        w = 0.4
        h = 0.04
        d = 0.3
        r = 0.05

        glTranslatef(x_pos, r, -y_pos)
        self.rotate(0, theta*180.0/numpy.pi - numpy.pi/2, 0)

        glPushMatrix()
        glTranslatef(0, h/2, 0)
        glColor3f(0.11, 0.56, 1.0)
        self.paint_cuboid(w, h, d)
        glPopMatrix()


        glPushMatrix()
        glTranslatef(w/2 + w/10, h/2, 0)
        glColor3f(1.0, 0.0, 0.0)
        self.paint_cuboid(w/10, h, d)
        glPopMatrix()

        glPushMatrix()
        glTranslatef(0, 0, d/2 + 0.01)
        glColor3f(0.4, 0.0, 1.0)
        self.paint_circle(r)
        glPopMatrix()

        glPushMatrix()
        glTranslatef(0, 0, -d/2 - 0.01)
        glColor3f(0.4, 0.0, 1.0)
        self.paint_circle(r)
        glPopMatrix()

        glPopMatrix()

        glPushMatrix()
        glBegin(GL_LINES)

        glColor3f(1.0, 0.0, 0.0)
        for j in range(len(self.path) - 1):
            x0 = self.path[j][0]
            y0 = self.path[j][1]
            x1 = self.path[j+1][0]
            y1 = self.path[j+1][1]
            glVertex3f(x0,  r, -y0)
            glVertex3f(x1,  r, -y1)

        glEnd()

        glPopMatrix()


        glfw.swap_buffers(self.window)
        glfw.poll_events()


        


    def rotate(self, angle_x, angle_y, angle_z):
        glRotatef(angle_x, 1.0, 0.0, 0.0)
        glRotatef(angle_y, 0.0, 1.0, 0.0)
        glRotatef(angle_z, 0.0, 0.0, 1.0)

   
    def paint_cuboid(self, width, height, depth):
        w = width/2.0
        h = height/2.0
        d = depth/2.0

        glBegin(GL_QUADS);        

        glVertex3f( w, h, -d)
        glVertex3f(-w, h, -d)
        glVertex3f(-w, h,  d)
        glVertex3f( w, h,  d)

        glVertex3f( w, -h,  d)
        glVertex3f(-w, -h,  d)
        glVertex3f(-w, -h, -d)
        glVertex3f( w, -h, -d)

        glVertex3f( w,  h, d)
        glVertex3f(-w,  h, d)
        glVertex3f(-w, -h, d)
        glVertex3f( w, -h, d)

        glVertex3f( w, -h, -d)
        glVertex3f(-w, -h, -d)
        glVertex3f(-w,  h, -d)
        glVertex3f( w,  h, -d)

        glVertex3f(-w,  h,  d)
        glVertex3f(-w,  h, -d)
        glVertex3f(-w, -h, -d)
        glVertex3f(-w, -h,  d)

        glVertex3f(w,  h, -d)
        glVertex3f(w,  h,  d)
        glVertex3f(w, -h,  d)
        glVertex3f(w, -h, -d)

        glEnd()

    def paint_circle(self, radius, steps = 32):
        pi = 3.141592654
        glBegin(GL_POLYGON)
        
        for i in range(steps):
            angle = i*2.0*pi/steps
            glVertex3f(radius*numpy.cos(angle), radius*numpy.sin(angle), 0.0)
        
        glEnd()