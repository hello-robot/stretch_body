# Based on the multiprocessing tutorial here:
# https://matplotlib.org/stable/gallery/misc/multiprocess_sgskip.html

import threading
import multiprocessing as mp
import time
import random

import matplotlib.pyplot as plt
import numpy as np


class ProcessPlotter:
    def __init__(self):
        self.time = []
        self.lift = []
        self.arm = []
        self.wrist_roll = []
        self.head_pan = []

    def terminate(self):
        plt.close('all')

    def call_back(self):
        while self.pipe.poll():
            command = self.pipe.recv()
            if command is None:
                self.terminate()
                return False
            else:
                self.time.append(command[0])
                self.lift.append(command[1])
                self.arm.append(command[2])
                self.wrist_roll.append(command[3])
                self.head_pan.append(command[4])
                self.ax[0].plot(self.time, self.lift, color='red')
                self.ax[1].plot(self.time, self.arm, color='green')
                self.ax[2].plot(self.time, self.wrist_roll, color='blue')
                self.ax[3].plot(self.time, self.head_pan, color='purple')
        self.fig.canvas.draw()
        return True

    def __call__(self, pipe):
        self.pipe = pipe
        self.fig, self.ax = plt.subplots(4)
        self.ax[0].set_ylim([0.0, 1.1])
        self.ax[1].set_ylim([0.0, 0.5])
        self.ax[2].set_ylim([-1.57, 1.57])
        self.ax[3].set_ylim([-4.04, 1.73])
        timer = self.fig.canvas.new_timer(interval=250)
        timer.add_callback(self.call_back)
        timer.start()

        plt.show()


class NBPlot:
    def __init__(self):
        self.plot_pipe, plotter_pipe = mp.Pipe()
        self.plotter = ProcessPlotter()
        self.plot_process = mp.Process(
            target=self.plotter, args=(plotter_pipe,), daemon=True)
        self.plot_process.start()
        self.start_time = time.time()

    def plot(self, lift_pos, arm_pos, roll_pos, pan_pos, finished=False):
        send = self.plot_pipe.send
        if finished:
            send(None)
        else:
            data = np.array([time.time() - self.start_time, lift_pos, arm_pos, roll_pos, pan_pos])
            send(data)


def main(flag):
    pl=NBPlot()
    while not flag.is_set():
        pl.plot(random.uniform(0.0, 1.1), random.uniform(0.0, 0.5), random.uniform(-1.57, 1.57), random.uniform(-4.04, 1.5))
        time.sleep(0.25)
    pl.plot(None, None, None, None, finished=True)


# if __name__ == '__main__':
#     shutdown_flag = threading.Event()
#     threading.Thread(target=main, args=(shutdown_flag,)).start()
#     threading.Thread(target=main, args=(shutdown_flag,)).start()
#     print('Press Ctrl-C to exit')
#     try:
#         while True:
#             pass
#     except:
#         pass
#     finally:
#         shutdown_flag.set()
#         time.sleep(1)


if __name__ == '__main__':
    shutdown_flag = threading.Event()
    threading.Thread(target=main, args=(shutdown_flag,)).start()
    threading.Thread(target=main, args=(shutdown_flag,)).start()
    time.sleep(5)
    shutdown_flag.set()