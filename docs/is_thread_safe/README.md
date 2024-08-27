# Is Stretch Body thread safe?

The short answer: no. This tutorial builds up to a few kinds of threading experiments that prove this, and hopefully will serve as a launching point for making this library thread-safe.

First, let's define what it means to be thread-safe. If we create an instance of the `Robot` class, we'd like to know if we can safely use it in Python threads. Using the `Robot`, there's two ways we interact with the underlying hardware: reading sensor data and writing motion commands. Our experiments will cover:

 1. Single threaded reading and writing
 2. Single writer, but multithreaded readers
 3. Multithreading for both reading and writing

Having multiple readers, even if only one thread can send commands, would be useful for visualization purposes.

## Writing

Let's create an instance of the `Robot`:

```python
import stretch_body.robot

r = stretch_body.robot.Robot()
assert r.startup()
assert r.is_homed()
```

Then create a "writer", that will send motion commands:

```python
import random

def write():
    r.lift.move_to(random.uniform(0.3, 1.1))
    r.arm.move_to(random.uniform(0.0, 0.4))
    r.end_of_arm.move_to('wrist_roll', random.uniform(-1.0, 1.0))
    r.head.move_to('head_pan', random.uniform(-1.0, 1.0))
    r.push_command()
```

Then let's define a writing-only runner, that sends motion commands at 10hz:

```python
def wo_runner():
    """write-only runner"""
    while True:
        write()
        time.sleep(0.1)
```

### Results

We can run `wo_runner()` in the main thread without any issues. But if we launch `wo_runner()` in a thread:

```python
import threading
threading.Thread(target=wo_runner).start()
```

The program exits with no error. Likely an exception is being thrown in the thread, but isn't being surfaced to the user. Surfacing this exception is the first step towards making this library thread-safe.

Notably, if we command out the lift and arm motion (leaving only the Dxl joints), the thread works fine.

```python
def write():
    # r.lift.move_to(random.uniform(0.3, 1.1)) # not thread-safe
    # r.arm.move_to(random.uniform(0.0, 0.4))  # not thread-safe
    r.end_of_arm.move_to('wrist_roll', random.uniform(-1.0, 1.0))
    r.head.move_to('head_pan', random.uniform(-1.0, 1.0))
    r.push_command()
```

## Reading

Let's create a "reader", that will plot the joint states for the four joints:

```python
def read(plotter):
    lift_pos = r.lift.status['pos']
    arm_pos = r.arm.status['pos']
    roll_pos = r.end_of_arm.get_joint('wrist_roll').status['pos']
    pan_pos = r.head.get_joint('head_pan').status['pos']
    plotter.plot(lift_pos, arm_pos, roll_pos, pan_pos)
```

Then let's define a reading-only runner, that will plot joint state at 10hz:

```python
def ro_runner():
    """read-only runner"""
    pl=NBPlot()
    while True:
        read(pl)
        time.sleep(0.1)
```

`NBPlot` is defined within `multiprocessing_plotter.py` in this folder. It enables Matplotlib to plot within threads.

### Results

We can run `ro_runner()` in the main thread without any issues, but if we launch `ro_runner()` in a thread:

```python
import threading
threading.Thread(target=ro_runner).start()
```

Once again, the program exits with no error. Surfacing the underlying error will be helpful to debug.

## Reading and Writing

For completion, we have `wr_runner()`:

```python
def rw_runner():
    """read-and-write runner"""
    pl=NBPlot()
    while True:
        write()
        read(pl)
        time.sleep(0.1)
```

Which runs into the same problems as above.

## Code

The code for this experiment lives in two files within this same directory:

 - `read_write.py`: which defines the reading and writing runners
 - `multiprocessing_plotter.py`: which is a helper library for plotting within Python threads. The logic does not impact these experiments.

## Takeaway

This document captures a few experiments around Stretch Body's thread safety. It shows that the library is currently not thread safe, and highlights some possible ways to start debugging how to make the library thread-safe.
