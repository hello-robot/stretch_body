import threading
import time

from stretch_body.trajectory import Trajectory


class TrajectoryManager:
    """Basic trajectory manager class that should be extended.

    Provides threading and utility functions for managing
    the execution of trajectories. This class is extended
    to support specific joint types.

    Attributes
    ----------
    traj_thread_freq : int
        the frequency in hz that the trajectory is pushed to hardware
    traj_thread_shutdown_flag : threading.Event
        signals shutdown to the trajectory pushing thread
    traj_thread : threading.Thread
        the trajectory pushing thread
    traj_threaded : bool
        if True, trajectory manager takes charge of pushing trajectory
    traj_start_time : float
        wall time at which trajectory execution began
    traj_curr_time : float
        wall time at which trajectory is being evaluated

    """

    def __init__(self):
        self.traj_thread_freq = 100
        self.traj_thread_shutdown_flag = threading.Event()
        self.traj_thread = None
        self.traj_threaded = False
        self.traj_start_time = None
        self.traj_curr_time = None
        self.trajectory = Trajectory()

    def _start_trajectory_thread(self):
        if self.traj_threaded:
            if self.traj_thread is not None:
                self.traj_thread_shutdown_flag.set()
                self.traj_thread.join()
            self.traj_thread = threading.Thread(target=self._push_trajectory_thread)
            self.traj_thread_shutdown_flag.clear()
            self.traj_thread.start()

    def _stop_trajectory_thread(self):
        if self.traj_threaded:
            self.traj_thread_shutdown_flag.set()
            self.traj_thread.join()

    def _push_trajectory_thread(self):
        while not self.traj_thread_shutdown_flag.is_set():
            ts = time.time()
            if not self.push_trajectory():
                return
            te = time.time()
            tsleep = max(0, (1 / self.traj_thread_freq) - (te - ts))
            if not self.traj_thread_shutdown_flag.is_set():
                time.sleep(tsleep)

    def duration_remaining(self):
        if len(self.trajectory) < 2:
            raise RuntimeError('Not enough points defined')
        traj_duration = self.trajectory.get_duration()
        if self.traj_start_time is None:
            return traj_duration
        if self.traj_curr_time is None:
            self.traj_curr_time = time.time()
        traj_elapased = self.traj_curr_time - self.traj_start_time
        return max(0.0, traj_duration - traj_elapased)
