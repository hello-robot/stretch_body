import rerun as rr
import rerun.blueprint as rrb
import time
import math
import socket

class RRplot:
    """Rerun timeseries plotter class"""
    def __init__(self, 
                 name,
                 open_browser:bool = False, 
                 server_memory_limit:str = "4GB",
                 view_range_s:int = 20,
                 web_port:int = 9090,
                 ws_port:int = 9877):
        """
        Args:
            name (str): name of the plotter
            open_browser (bool): open browser on startup
            server_memory_limit (str): memory limit for the server
            view_range_s (int): range of the last data window displayed in seconds
            web_port (int): port for the web server
            ws_port (int): port for the websocket server
        """
        self.__reg_keys = {} 
        self.__name = name
        self.__view_range_s = view_range_s
        self.__local_ip = self.get_local_ip()
        if self.__local_ip is not None:
            print('='*50)
            print(f"Rerun plotter running at http://{self.__local_ip}:{web_port}/?url=ws://{self.__local_ip}:{ws_port}")
            print('='*50)

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

    def get_local_ip(self):
        """
        Get the local IP address of the system that can be accessed by remote computers.
        Returns:
            str: Local IP address.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Connect to an external address to determine the local IP
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        finally:
            s.close()
        return local_ip
        
    def register(self, key:str, color_idx:int, range:tuple=None):
        """
        Register a datafield key to be logged
        Args:
            key (str): key to be registered
            color_idx (int): index of the color in the color palette
            range (tuple): range of the datafield displayed
        """
        if color_idx >= len(self.color_palette):
            raise ValueError(f"Color index out of range, max index value is {len(self.color_palette) - 1}")
        self.__reg_keys[key] = {'color':color_idx, 'range':range}
        rr.log(f"{self.__name}/{key}", rr.SeriesLine(color=self.color_palette[color_idx], name=key, width=2), static=True)


    def log_scalar(self, key:str, value:float|int):
        """
        Log a scalar value to the registered key
        Args:
            key (str): registered key
            value (float|int): scalar value to be logged
        """
        if not isinstance(value, (int, float)):
            raise ValueError(f"Value must be a scalar (int or float), got {type(value)}")
        if key not in self.__reg_keys.keys():
            raise ValueError(f"Key {key} not registered")
        if self._start_ts is None:
            self._start_ts = time.perf_counter()
        rr.set_time_seconds("realtime", time.perf_counter() - self._start_ts)
        rr.log(f"{self.__name}/{key}", rr.Scalar(value))
    
    def setup_blueprint(self, collapse_panels:bool=False):
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


if __name__ == "__main__":
    n = 5  # Number of sine wave series
    rrplot = RRplot(name="SineWaveVisualizer")

    # Register n sine wave series
    for i in range(n):
        rrplot.register(key=f"sine_wave_{i}", color_idx=i % len(rrplot.color_palette), range=(-1.5, 1.5))

    rrplot.setup_blueprint(collapse_panels=False)
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        for i in range(n):
            value = math.sin(elapsed_time + (i * math.pi / n))  # Phase shift for each series
            rrplot.log_scalar(key=f"sine_wave_{i}", value=value)
        time.sleep(1/30)