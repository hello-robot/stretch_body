#! /usr/bin/env python
import time
from stretch_body.device import Device
from stretch_body.hello_utils import *
import numpy as np

class HardwareClockManager(Device):
    """
    Attempts to keep the hardware clock from the Pimu/Wacc aligned with the PC clock. This uses:
    * TRS: Time from commanding a HW clock sync to it being triggered on Pimu/Wacc (estimate at 50% of roundtrip time) (calibrated at factory)
    * SKEW: Current difference between PC and HW clocks
    The skew is averaged over a large time window and then added in the conversion from HW clock to PC clock
    Todo: a calibration / closed loop tracking of the skew
    """
    def __init__(self,hw_device, hw_device_name):
        Device.__init__(self)
        self.params = self.robot_params[hw_device_name]
        self.hw_device=hw_device
        self.hw_device_name=hw_device_name
        self.timestamp_hw_base = SystemTimestamp()
        self.ts_skew_start = SystemTimestamp()
        self.skew_history=[]
        self.skew_avg=0
        self.adj_skew=0

    def pretty_print(self):
        print('------ '+self.hw_device_name+' Clock Manager ------')
        print('HW clock slower than PC (usecs): '+str(self.skew_avg))
        print('Adjusted HW clock slower than PC (usecs): ' + str(self.adj_skew))

    def start_skew_measure(self):
        # Call at approximately 20 hz from within a status_sync call (so the timestamp_clock_sync is fresh)
        self.ts_skew_start = SystemTimestamp().from_wall_time()

    def end_skew_measure(self):
        dt_total_pc =self.ts_skew_start + SystemTimestamp().from_usecs(self.params['trs']) - self.timestamp_hw_base #PC time since start until status sync triggered on Pimu
        dt_total_hw = self.hw_device.status['timestamp_status_sync'] #Hw time since start until status sync triggered on Pimu/Wacc
        e = (dt_total_pc - dt_total_hw).to_usecs()
        self.skew_history.append(e)
        if len(self.skew_history) >= self.params['n_slew_history']:
            self.skew_history = self.skew_history[1:]
        self.skew_avg = float(np.array(self.skew_history).mean())

        dt_total_hw_adj = self.HW_to_PC_timestamp(self.hw_device.status['timestamp_status_sync'])-self.timestamp_hw_base
        self.adj_skew = (dt_total_pc - dt_total_hw_adj).to_usecs()


    def HW_to_PC_timestamp(self,ts_hw):
        if self.params['use_skew_compensation']:
            hw_usecs = ts_hw.to_usecs() + self.skew_avg
            return self.timestamp_hw_base + SystemTimestamp().from_usecs(hw_usecs)
        else:
            return self.timestamp_hw_base + ts_hw

    def PPM_to_pct(self,ppm):
        return ppm/1000000

    def pct_to_PPM(self,pct):
        return pct*1000000

    def zero_HW_clock(self):
        #Resets the clock on the Pimu/Wacc and estimates the time this as triggered as 50% of TRS
        t0=SystemTimestamp().from_wall_time()
        self.hw_device.trigger_clock_zero()
        self.timestamp_hw_base = t0+SystemTimestamp().from_usecs(self.params['trs'])


    def sample_TRS(self):
        t0 = time.time()
        self.hw_device.trigger_status_sync()
        t1 = time.time()
        dt = (t1 - t0) * 1000000 * 0.5 #Roundtrip time to do the clock sync (usecs)
        return dt

