from __future__ import print_function
from matplotlib.widgets import Slider, Button
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# https://github.com/stsievert/python-drawnow
#pip install drawnow
from drawnow import drawnow

from stretch_body.hello_utils import *
import numpy as np
import time


class Scope:
    """
    Simple oscilloscope visualization of a data stream
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


class TrajectoryScope:

    def __init__(self, x, y, v, yrange=None, vrange=None, sense_frequency=100,
                 title='Interactive Scope', xlabel='Time (s)', ylabel='Data'):
        """Interactive scope for cubic trajectory planning and visualization.

        Allows interactive cubic trajectory planning using waypoints and sliders.
        Execution and reset buttons start and stop trajectory tracking.
        While trajectory executes, joint position stream is plotted over the
        planned stream. Trajectory can be modified while being executed.

        Parameters
        ----------
        x : list(float)
            Starting x axis waypoints. Generally is time (seconds)
        y : list(float)
            Starting y axis waypoints. Generally is joint position (unitless)
        v : list(float)
            Starting velocity waypoints for the trajectory
        yrange : tuple(float, float)
            Two tuple representing joint limits
        vrange : tuple(float, float)
            Two tuple representing joint velocity limits
        sense_frequency : int
            Frequency (hz) at which scope measures joint position
        title : str
            Title of the scope window
        xlabel: str
            Label for the x axis
        ylable: str
            Label for the y axis

        Attributes
        ----------
        initx, inity, initv : list(float)
            initial waypoints for resetting graph
        sensex, sensey : list(float)
            used to plot measured joint position over time
        epsilon : int
            clicking radius for waypoint
        """
        self.x, self.initx = x[:], x[:]
        self.y, self.inity = y[:], y[:]
        self.v, self.initv = v[:], v[:]
        self.sensex = []
        self.sensey = []
        self.num_points = min(len(x), len(y), len(v))
        self.yrange = yrange if yrange is not None else (0, 10)
        self.vrange = vrange if vrange is not None else (0, 10)
        self.epsilon = 15
        self.pind = None

        # Setup plot and widgets
        widget_loc = lambda x: plt.axes([0.84, 0.8-((x)*0.05), 0.12, 0.02])
        self.fig, self.axes = plt.subplots(1, 1, figsize=(9.0, 8.0), sharex=True)
        self.fig.subplots_adjust(right=0.8)
        self.fig.canvas.set_window_title(title)
        self.axes.set_yscale('linear')
        self.axes.set_ylim(min(self.yrange) - 0.75, max(self.yrange) + 0.75)
        self.axes.axhspan(min(self.yrange) - 2 ** 32, min(self.yrange), facecolor='0.2', alpha=0.5)
        self.axes.axhspan(max(self.yrange), max(self.yrange) + 2 ** 32, facecolor='0.2', alpha=0.5)
        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)
        self.axes.grid(True)
        self.reset_button = Button(widget_loc(self.num_points), 'Reset')
        self.reset_button.on_clicked(self._reset)
        self.exec_button = Button(widget_loc(self.num_points + 1), 'Execute', color='limegreen', hovercolor='lightgreen')
        self.exec_button.on_clicked(self._execute)
        self.sliders = []
        for i in range(self.num_points):
            s = Slider(widget_loc(i), 'v{0}'.format(i), self.vrange[0], self.vrange[1], valinit=self.initv[i])
            s.on_changed(self._update)
            self.sliders.append(s)
        self.fig.canvas.mpl_connect('button_press_event', self._mouse_down_cb)
        self.fig.canvas.mpl_connect('button_release_event', self._mouse_up_cb)
        self.fig.canvas.mpl_connect('motion_notify_event', self._mouse_move_cb)
        self.anim = animation.FuncAnimation(self.fig, self._animate, interval=int(1000 / sense_frequency))
        self.executing = False

        # Plot data
        self.m, = self.axes.plot(self.initx, self.inity, color='gray', linestyle='--', label='Trajectory')
        self.s, = self.axes.plot(self.sensex, self.sensey, color='limegreen', label='Sense')
        self.l, = self.axes.plot(self.initx, self.inity, color='tab:blue', linestyle='none', marker='o', markersize=12)
        self.lfirst, = self.axes.plot(self.initx[0], self.inity[0], color='0.6', linestyle='none', marker='o', markersize=12)
        self.axes.legend(loc="upper right")
        self._update(self)

    def start(self, exec_func, sense_func, waypoints_change_func, stop_func):
        """Starts the scope with four callback functions

        Parameters
        ----------
        exec_func : func
            Called when execute button in scope is pressed
        sense_func : func
            Called at ``sense_frequency`` to plot joint position
        waypoints_change_func : func
            Called with updated waypoints when user changes them
        stop_func : func
            Called when stop button in scope is pressed
        """
        self.exec_func = exec_func
        self.sense_func = sense_func
        self.waypoints_change_func = waypoints_change_func
        self.stop_func = stop_func
        plt.show()

    def _animate(self, i):
        sense = self.sense_func()
        if sense:
            self.sensex.append(sense[0])
            self.sensey.append(sense[1])
            self._update(self)

    def _execute(self, e):
        if not self.executing:
            self.exec_func(self.x, self.y, self.v)
            self.anim.event_source.start()
            self.executing = True

    def _reset(self, e):
        self.anim.event_source.stop()
        stopped_pos = self.stop_func()
        self.inity[0] = stopped_pos
        self.executing = False
        self.sensex = []
        self.sensey = []
        self.x = self.initx[:]
        self.y = self.inity[:]
        self.v = self.initv[:]
        for i in range(self.num_points):
            self.sliders[i].set_val(self.initv[i])
        self._update(self)

    def _update(self, e):
        for i in range(self.num_points):
            self.v[i] = self.sliders[i].val
        if self.executing and self.pind is not None or \
           self.executing and isinstance(e, np.float64):
            self.waypoints_change_func(self.x, self.y, self.v)
        self.lfirst.set_xdata(self.x[0])
        self.lfirst.set_ydata(self.y[0])
        self.l.set_xdata(self.x)
        self.l.set_ydata(self.y)
        self.s.set_xdata(self.sensex)
        self.s.set_ydata(self.sensey)
        splinex = []
        spliney = []
        a = list(zip(self.x, self.y, self.v))
        for (i, f) in zip(a, a[1:]):
            seg = generate_cubic_polynomial(i, f)
            segx = np.arange(i[0], f[0], 0.05)
            for t in segx:
                splinex.append(t)
                spliney.append(evaluate_polynomial_at(seg[1:], t - i[0])[0])
        self.m.set_xdata(splinex)
        self.m.set_ydata(spliney)
        self.fig.canvas.draw_idle()

    def _mouse_down_cb(self, e):
        if e.inaxes is None:
            return
        if e.button != 1:
            return
        t = self.axes.transData.inverted()
        tinv = self.axes.transData
        xy = t.transform([e.x, e.y])
        xr = np.reshape(self.x, (np.shape(self.x)[0], 1))
        yr = np.reshape(self.y, (np.shape(self.y)[0], 1))
        xy_vals = np.append(xr, yr, 1)
        xyt = tinv.transform(xy_vals)
        xt, yt = xyt[:, 0], xyt[:, 1]
        d = np.hypot(xt - e.x, yt - e.y)
        indseq, = np.nonzero(d == d.min())
        ind = indseq[0]
        if d[ind] >= self.epsilon:
            ind = None
        self.pind = ind

    def _mouse_up_cb(self, e):
        if e.button != 1:
            return
        self.pind = None

    def _mouse_move_cb(self, e):
        if self.pind is None:
            return
        if e.inaxes is None:
            return
        if e.button != 1:
            return
        if self.pind == 0:
            return
        if self.pind == len(self.x) - 1:
            prelimit = self.x[self.pind - 1] + 0.5
            postlimit = self.x[self.pind] + 10000
        else:
            prelimit = self.x[self.pind - 1] + 0.5
            postlimit = self.x[self.pind + 1] - 0.5

        if e.xdata > prelimit and e.xdata < postlimit:
            self.x[self.pind] = e.xdata
        if e.ydata > min(self.yrange) and e.ydata < max(self.yrange):
            self.y[self.pind] = e.ydata
        self._update(self)


class Scope4:
    """
   Simple 4ch oscilloscope visualization of a data stream
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

