from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.device import Device
from stretch_body.robot_params import RobotParams
from stretch_body.gripper_conversion import GripperConversion
import rerun as rr
import rerun.blueprint as rrb
import time


class RRplot:
    def __init__(self, 
                 name,
                 open_browser = False, 
                 server_memory_limit="4GB",
                 view_range_s=20,
                 web_port=9090,
                 ws_port=9877):
        self.__reg_keys = {} 
        self.__name = name
        self.__view_range_s = view_range_s
        self.color_palette = [
            [255, 0, 0],    # Red
            [0, 255, 255],  # Cyan
            [128, 0, 0],    # Maroon
            [0, 128, 0],    # Dark Green
            [0, 0, 128],    # Navy
            [128, 128, 0],  # Olive
            [128, 0, 128],  # Purple
            [0, 128, 128],  # Teal
            [192, 192, 192],# Silver
            [128, 128, 128],# Gray
            [0, 0, 0]       # Black
        ]
        rr.init(name)
        rr.serve_web(open_browser=open_browser,
                     server_memory_limit=server_memory_limit,
                     web_port=web_port,
                     ws_port=ws_port)
        self.__blueprint = None
        self._start_ts = None
    
    def register(self, key, color_idx, range:tuple=None):
        """
        Register a datafield key to be logged"""
        if color_idx >= len(self.color_palette):
            raise ValueError(f"Color index out of range, max index is {len(self.color_palette) - 1}")
        self.__reg_keys[key] = {'color':color_idx, 'range':range}
        rr.log(f"{self.__name}/{key}", rr.SeriesLine(color=self.color_palette[color_idx], name=key, width=2), static=True)


    def log_scalar(self, key, value):
        # if self.__blueprint is None:
        #     raise ValueError("Blueprint not set up")
        if not isinstance(value, (int, float)):
            raise ValueError(f"Value must be a scalar (int or float), got {type(value)}")
        if key not in self.__reg_keys.keys():
            raise ValueError(f"Key {key} not registered")
        if self._start_ts is None:
            self._start_ts = time.perf_counter()
        rr.set_time_seconds("realtime", time.perf_counter() - self._start_ts)
        rr.log(f"{self.__name}/{key}", rr.Scalar(value))
    
    def setup_blueprint(self, collapse_panels: bool=False):
        """Setup the blueprint for the visualizer
        Args:
            collapse_panels (bool): fully hides the blueprint/selection panels,
                                    and shows the simplified time panel
        """
        views = []
        for k in self.__reg_keys.keys():
            range = rrb.VisibleTimeRange(
                "realtime",
                start=rrb.TimeRangeBoundary.cursor_relative(seconds=-1*abs(self.__view_range_s)),
                end=rrb.TimeRangeBoundary.infinite(),
            )
            views.append(rrb.TimeSeriesView(name=k, 
                                            origin=[f"{self.__name}/{k}"], 
                                            time_ranges=[range,],
                                            plot_legend=rrb.PlotLegend(visible=True),
                                            axis_y=rrb.ScalarAxis(range=self.__reg_keys[k]['range'], zoom_lock=False)
                                            ),
                                            )
        my_blueprint = rrb.Blueprint(
            rrb.Vertical(contents=views),
            collapse_panels=collapse_panels,
        )
        self.__blueprint = my_blueprint
        rr.send_blueprint(self.__blueprint)


class StretchGripper(DynamixelHelloXL430):
    """
    API to the Stretch Gripper
    The StretchGripper motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the gripper is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 to +200 (fully open)
    The fully open value (self.pct_max_open) is dependent on mechanical design of the gripper
    which changes depending on the robot generation (RE1, RE2, SE3, etc)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None, usb=None, name='stretch_gripper'):
        DynamixelHelloXL430.__init__(self, name, chain, usb)
        self.status['pos_pct']= 0.0
        self.pct_max_open=self.world_rad_to_pct(self.ticks_to_world_rad(self.params['range_t'][1])) #May be a bit greater than 50 given non-linear calibration
        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}
        self.status['gripper_conversion'] = {'aperture_m':0.0,
                                             'finger_rad':0.0,
                                             'finger_effort':0.0,
                                             'finger_vel':0.0}
        self.gripper_conversion = GripperConversion(self.params['gripper_conversion'])

        self.plt = RRplot("gripper")
        self.plt.register("pos", 0, (-10.5,10.5))
        self.plt.register("effort", 1, (-100,100))
        self.plt.register("vel", 2)
        self.plt.register("temp", 3, (25, 100))
        self.plt.register("stall_overload", 5, (0,1))
        self.plt.register("overload_error", 6, (0,1))
        self.plt.register("voltage", 7, (9,14))
        self.plt.register("stalled", 8, (0,1))
        self.plt.setup_blueprint()

    def startup(self, threaded=True):
        return DynamixelHelloXL430.startup(self, threaded=threaded)

    def home(self,move_to_zero=True):
        DynamixelHelloXL430.home(self,single_stop=True,move_to_zero=move_to_zero,delay_at_stop=2.25)

    def pretty_print(self):
        print('--- StretchGripper ----')
        print("Position (%)",self.status['pos_pct'])
        DynamixelHelloXL430.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self,pct, v_r=None, a_r=None):
        """
        pct: commanded absolute position (Pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_r=self.pct_to_world_rad(pct)
        DynamixelHelloXL430.move_to(self,x_des=x_r, v_des=v_r, a_des=a_r)


    def move_by(self,delta_pct,v_r=None,a_r=None):
        """
        delta_pct: commanded incremental motion (pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.pull_status() #Ensure up to date
        self.move_to(self.status['pos_pct']+delta_pct,v_r,a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        DynamixelHelloXL430.pull_status(self,data)
        self.status['pos_pct']=self.world_rad_to_pct(self.status['pos'])
        self.status['gripper_conversion']=self.gripper_conversion.get_status(self.status)

        self.plt.log_scalar("pos", self.status['pos'])
        self.plt.log_scalar("effort", self.status['effort'])
        self.plt.log_scalar("vel", self.status['vel'])
        self.plt.log_scalar("temp", self.status['temp'])
        self.plt.log_scalar("stall_overload", int(self.status['stall_overload']))
        self.plt.log_scalar("overload_error", int(self.status['overload_error']))
        self.plt.log_scalar("voltage", self.motor.get_voltage()*0.1)
        self.plt.log_scalar("stalled", int(self.status['stalled']))

    def pct_to_world_rad(self,pct):
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t=((-1*pct)-100)*pct_to_tick
        r = DynamixelHelloXL430.ticks_to_world_rad(self, t)
        return r

    def world_rad_to_pct(self,r):
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t = DynamixelHelloXL430.world_rad_to_ticks(self,r)
        pct = -1*((t / pct_to_tick)+100)
        return pct

    def step_sentry(self, robot):
        """This sentry attempts to prevent the gripper servo from overheating during a prolonged grasp
        When the servo is stalled and exerting an effort above a threshold it will command a 'back off'
        position (slightly opening the grasp). This reduces the PID steady state error and lowers the
        commanded current. The gripper's spring design allows it to retain its grasp despite the backoff.
        """
        DynamixelHelloXL430.step_sentry(self, robot)
        if self.hw_valid and self.robot_params['robot_sentry']['stretch_gripper_overload'] and not self.is_homing:
            if self.status['stall_overload']:
                if self.in_vel_mode:
                    self.enable_pos()
                if self.status['effort'] < 0: #Only backoff in open direction
                    self.logger.debug('Backoff at stall overload')
                    self.move_by(self.params['stall_backoff'])

class StretchGripper3(StretchGripper):
    """
        Wrapper for version 3 (for DW3)
    """
    def __init__(self, chain=None, usb=None):
        StretchGripper.__init__(self, chain, usb,'stretch_gripper')
