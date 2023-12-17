#!/usr/bin/env python3
import os
import signal
import time
from filelock import FileLock, Timeout

pid_file = "/tmp/stretch_body_robot_pid.txt"
file_lock = FileLock(f"{pid_file}.lock")
try:
    file_lock.acquire(timeout=1)
    file_lock.release()
except Timeout:
    with open(pid_file, 'r') as f:
        tokill_pid = int(f.read())
        # send SIGTERM a few times some processes (e.g. ipython)
        # try to stall on exit
        os.kill(tokill_pid, signal.SIGTERM)
        time.sleep(0.2)
        os.kill(tokill_pid, signal.SIGTERM)
        time.sleep(0.2)
        os.kill(tokill_pid, signal.SIGTERM)
finally:
    print('Done!')
