#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.arm
import stretch_body.head
import stretch_body.lift
import stretch_body.wrist_yaw
from stretch_body.hello_utils import deg_to_rad
from stretch_body.trajectories import Spline, Waypoint
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation
import numpy as np

import time
import argparse

PRELOADED_TRAJECTORIES = {
    'arm': {
        '1': {
            't': [10.0, 20.0],
            'p': [0.5, 0.0],
            'v': [0.0, 0.0]
        },
        '2': {
            't': [3.0, 6.0, 9.0],
            'p': [0.1, 0.3, 0.15],
            'v': [0.0, 0.0, 0.0]
        },
        '3': {
            't': [30.0, 60.0],
            'p': [0.5, 0.0],
            'v': [0.0, -0.06]
        }
    },
    'head_tilt': {
        '1': {
            't': [3.0, 7.0, 10.0],
            'p': [deg_to_rad(-90.0), deg_to_rad(20.0), deg_to_rad(0.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
        },
        '2': {
            't': [3.0, 6.0],
            'p': [deg_to_rad(-50.0), deg_to_rad(0.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(0.0)]
        },
        '3': {
            't': [30.0, 60.0],
            'p': [deg_to_rad(-90.0), deg_to_rad(0.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(10.0)]
        }
    },
    'head_pan': {
        '1': {
            't': [4.0, 8.0, 10.0],
            'p': [deg_to_rad(-180.0), deg_to_rad(60.0), deg_to_rad(0.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
        },
        '2': {
            't': [3.0, 6.0],
            'p': [deg_to_rad(-100.0), deg_to_rad(0.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(0.0)]
        },
        '3': {
            't': [30.0, 60.0],
            'p': [deg_to_rad(-180.0), deg_to_rad(0.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(10.0)]
        }
    },
    'lift': {
        '1': {
            't': [15.0, 30.0],
            'p': [0.9, 0.2],
            'v': [0.0, 0.0]
        },
        '2': {
            't': [3.0, 6.0, 9.0],
            'p': [0.3, 0.4, 0.2],
            'v': [0.0, 0.0, 0.0]
        },
        '3': {
            't': [30.0, 60.0],
            'p': [0.9, 0.2],
            'v': [0.0, -0.08]
        }
    },
    'wrist_yaw': {
        '1': {
            't': [5.0, 10.0],
            'p': [deg_to_rad(-45.0), deg_to_rad(90.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(10.0)]
        },
        '2': {
            't': [3.0, 6.0, 9.0],
            'p': [deg_to_rad(40.0), deg_to_rad(-40.0), deg_to_rad(90.0)],
            'v': [deg_to_rad(90.0), deg_to_rad(-90.0), deg_to_rad(0.0)]
        },
        '3': {
            't': [30.0, 60.0],
            'p': [deg_to_rad(0.0), deg_to_rad(90.0)],
            'v': [deg_to_rad(0.0), deg_to_rad(10.0)]
        }
    }
}

class TrajectoryScope:
    def __init__(self, joint_name, preloaded_key):
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
        self.joint_name = joint_name
        sense_frequency = 15

        # Setup Stretch Connection
        if 'head' in joint_name:
            self.device = stretch_body.head.Head()
            self.traj_man = self.device.get_joint(joint_name)
            self.yrange = list(map(self.traj_man.ticks_to_world_rad, self.traj_man.params['range_t']))
            vrange = (-3.0, 3.0)

            proper_name = self.joint_name.replace('_', ' ').title()
            units = 'rad'
        elif joint_name == 'arm':
            self.device = stretch_body.arm.Arm()
            self.traj_man = self.device
            self.yrange = tuple(self.device.params['range_m'])
            vel_m = self.device.params['motion']['trajectory_max']['vel_m']
            vrange = (-vel_m, vel_m)
            proper_name = 'Arm'
            units = 'm'
        elif joint_name == 'lift':
            self.device = stretch_body.lift.Lift()
            self.traj_man = self.device
            self.yrange = tuple(self.device.params['range_m'])
            vel_m = self.device.params['motion']['trajectory_max']['vel_m']
            vrange = (-vel_m, vel_m)
            proper_name = 'Lift'
            units = 'm'
        elif joint_name == 'wrist_yaw':
            self.device = stretch_body.wrist_yaw.WristYaw()
            self.traj_man = self.device
            self.yrange = list(map(self.device.ticks_to_world_rad, self.device.params['range_t']))
            vrange = (-3.0, 3.0)
            proper_name = 'Wrist Yaw'
            units = 'rad'
        else:
            class X:
                def __init__(self):
                    self.status = {'pos': 1.0}

            self.traj_man = X()
            proper_name = 'unknown'
            units = '?'
            joint_name = 'head_tilt'
            self.yrange = (-3, 3)
            vrange = (-3, 3)

        self.device.startup()
        self.device.pull_status()

        initial_traj = PRELOADED_TRAJECTORIES[joint_name][preloaded_key]
        x = [0.0] + initial_traj['t']
        y = [self.traj_man.status['pos']] + initial_traj['p']
        v = [0.0] + initial_traj['v']
        self.x, self.initx = x[:], x[:]
        self.y, self.inity = y[:], y[:]
        self.v, self.initv = v[:], v[:]
        self.sensex = []
        self.sensey = []
        self.num_points = min(len(x), len(y), len(v))

        def widget_loc(x):
            return plt.axes([0.84, 0.8 - ((x) * 0.05), 0.12, 0.02])
        # Setup plot and widgets
        self.fig, self.axes = plt.subplots(1, 1, figsize=(9.0, 8.0), sharex=True)
        self.fig.subplots_adjust(right=0.8)
        self.fig.canvas.manager.set_window_title('{} Trajectory'.format(proper_name))
        self.axes.set_yscale('linear')
        self.axes.set_ylim(min(self.yrange) - 0.75, max(self.yrange) + 0.75)
        self.axes.axhspan(min(self.yrange) - 2 ** 32, min(self.yrange), facecolor='0.2', alpha=0.5)
        self.axes.axhspan(max(self.yrange), max(self.yrange) + 2 ** 32, facecolor='0.2', alpha=0.5)
        self.axes.set_xlabel('Time (s)')
        self.axes.set_ylabel('{} Joint Range ({})'.format(proper_name, units))
        self.axes.grid(True)

        self.epsilon = 15
        self.pind = None

        self.reset_button = Button(widget_loc(self.num_points), 'Reset')
        self.reset_button.on_clicked(self._reset)
        self.exec_button = Button(widget_loc(self.num_points + 1), 'Execute', color='limegreen',
                                  hovercolor='lightgreen')
        self.exec_button.on_clicked(self._execute)
        self.sliders = []
        for i in range(self.num_points):
            s = Slider(widget_loc(i), 'v{0}'.format(i), vrange[0], vrange[1], valinit=self.initv[i])
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
        self.lfirst, = self.axes.plot(self.initx[0], self.inity[0], color='0.6', linestyle='none', marker='o',
                                      markersize=12)
        self.axes.legend(loc='upper right')
        self._update(self)

        plt.show()

    def _update(self, e):
        for i in range(self.num_points):
            self.v[i] = self.sliders[i].val
        if self.executing and self.pind is not None or \
           self.executing and isinstance(e, np.float64):

           self.traj_man.trajectory.clear()
           for t, p, v in zip(self.x, self.y, self.v):
               wp = Waypoint(t, p, v)
               self.traj_man.trajectory.add_waypoint(wp)

        self.lfirst.set_xdata(self.x[0])
        self.lfirst.set_ydata(self.y[0])
        self.l.set_xdata(self.x)
        self.l.set_ydata(self.y)
        self.s.set_xdata(self.sensex)
        self.s.set_ydata(self.sensey)
        waypoints = [Waypoint(t, p, v) for t, p, v in zip(self.x, self.y, self.v)]
        spline = Spline(waypoints)
        splinex = np.arange(self.x[0], self.x[-1], 0.05)
        spliney = [spline.evaluate_at(t)[0] for t in splinex]
        self.m.set_xdata(splinex)
        self.m.set_ydata(spliney)
        self.fig.canvas.draw_idle()

    def _animate(self, i):
        self.device.pull_status()
        if self.traj_man.is_trajectory_active():
            # self.traj_man.push_trajectory()
            self.sensex.append(self.traj_man.get_trajectory_elapsed())
            self.sensey.append(self.traj_man.status['pos'])
            self._update(self)

    def _execute(self, e):
        if not self.executing:
            for t, p, v in zip(self.x, self.y, self.v):
                wp = Waypoint(t, p, v)
                self.traj_man.trajectory.add_waypoint(wp)

            self.traj_man.follow_trajectory(move_to_start_point=False)
            self.anim.event_source.start()
            self.executing = True

    def _reset(self, e):
        self.anim.event_source.stop()

        self.traj_man.stop_trajectory()
        self.traj_man.trajectory.clear()
        time.sleep(0.25)
        self.device.pull_status()
        stopped_pos = self.traj_man.status['pos']
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

    def _mouse_down_cb(self, e):
        if e.inaxes is None:
            return
        if e.button != 1:
            return
        tinv = self.axes.transData
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test out trajectories from a GUI.')
    parser.add_argument('joint', choices=['head_pan', 'head_tilt', 'arm', 'lift', 'wrist_yaw'])
    parser.add_argument('-p', '--preloaded_traj', help='Load one of three predefined trajectories',
                        choices=['1', '2', '3'], default='1')
    args = parser.parse_args()

    scope = TrajectoryScope(args.joint, args.preloaded_traj)
