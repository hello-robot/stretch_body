from __future__ import print_function
import matplotlib.pyplot as plt
from drawnow import drawnow
import time


class Scope:
    """
    Simple oscilliscope visualization of a data stream
    """
    def __init__(self,num_points=100,yrange=None,title='Scope'):
        plt.ion()  # enable interactivity
        self.y = None
        self.num_points=num_points
        self.fig = plt.figure()
        self.fig.canvas.set_window_title(title)
        self.yrange=yrange
    def step_display(self,new_val):
        if self.y is None:
            self.y=[new_val] * self.num_points
        self.y.append(new_val)
        self.y=self.y[1:]
        drawnow(self.make_fig)
    def draw_array(self,v):
        self.y=v
        drawnow(self.make_fig)
    def make_fig(self):
        plt.plot(self.y)
        if self.yrange is not None:
            plt.ylim(self.yrange[0], self.yrange[1])
    def close(self):
        pass
    def savefig(self,filename):
        plt.savefig(filename)

class Scope4:
    """
   Simple 4ch oscilliscope visualization of a data stream
   """
    def __init__(self,num_points=100,yrange=None,title='Scope'):
        plt.ion()  # enable interactivity
        self.y1 = None
        self.y2 = None
        self.y3 = None
        self.y4 = None
        self.num_points=num_points
        self.fig = plt.figure()
        self.fig.canvas.set_window_title(title)
        self.yrange=yrange
    def savefig(self,filename):
        plt.savefig(filename)
    def step_display(self,y1,y2,y3,y4):
        if self.y1 is None:
            self.y1=[y1] * self.num_points
        if self.y2 is None:
            self.y2=[y2] * self.num_points
        if self.y3 is None:
            self.y3=[y3] * self.num_points
        if self.y4 is None:
            self.y4=[y4] * self.num_points
        self.y1.append(y1)
        self.y1=self.y1[1:]
        self.y2.append(y2)
        self.y2 = self.y2[1:]
        self.y3.append(y3)
        self.y3 = self.y3[1:]
        self.y4.append(y4)
        self.y4 = self.y4[1:]
        drawnow(self.make_fig)

    def draw_array(self,y1,y2,y3,y4):
        self.y1=  y1[:]
        self.y2 = y2[:]
        self.y3 = y3[:]
        self.y4 = y4[:]
        drawnow(self.make_fig)

    def draw_array_xy(self,x1,x2,x3,x4,y1,y2,y3,y4):
        self.x1 = x1[:]
        self.x2 = x2[:]
        self.x3 = x3[:]
        self.x4 = x4[:]
        self.y1 = y1[:]
        self.y2 = y2[:]
        self.y3 = y3[:]
        self.y4 = y4[:]
        drawnow(self.make_fig_xy)

    def make_fig_xy(self):
        plt.plot(self.x1,self.y1)
        plt.plot(self.x2,self.y2)
        plt.plot(self.x3,self.y3)
        plt.plot(self.x4,self.y4)
        if self.yrange is not None:
            plt.ylim(self.yrange[0], self.yrange[1])

    def make_fig(self):
        plt.plot(self.y1)
        plt.plot(self.y2)
        plt.plot(self.y3)
        plt.plot(self.y4)
        if self.yrange is not None:
            plt.ylim(self.yrange[0], self.yrange[1])
    def close(self):
        pass

if __name__ == '__main__':
    import numpy as np
    s=Scope4(yrange=[0,1],title='Random')
    for i in range(100):
        s.step_display(np.random.random(),np.random.random(),np.random.random(),np.random.random())
        time.sleep(0.02)
    s.close()

