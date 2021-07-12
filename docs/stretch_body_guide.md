![](./images/banner.png)
# Working with Stretch Body
The Stretch_Body package provides a pure Python SDK to the Stretch RE1 hardware.  

The package is available on [Git and installable via Pip](https://github.com/hello-robot/stretch_body).

It encapsulates

* Mobile base 
* Arm 
* Lift 
* Head actuators
* Wrist and tool actuators
* Wrist accelerometer and Arduino
* Base power and IMU board

The robot's 3rd party hardware devices are intended to be accessed through ROS and not Stretch_Body. However, it is possible to directly access this hardware through open-source Python packages:

* Laser range finder:  [rplidar](https://github.com/SkoltechRobotics/rplidar)
* Respeaker: [respeaker_python_library](https://github.com/respeaker/respeaker_python_library)
* D435i: [pyrealsense2](https://pypi.org/project/pyrealsense2/)

The Stretch_Body package is intended for advanced users who prefer to not use ROS to control the robot. It assumes a moderate level of experience programming robot sensors and actuators.


## Robot Interface

The primary developer interface to  Stretch_Body is the [Robot class](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py).  

As an example, the Python script below prints all Robot sensor and state data to the console every 250ms. 

```python
import time
import stretch_body.robot

robot=stretch_body.robot.Robot()
robot.startup()

for i in range(10):
	robot.pretty_print()
	time.sleep(0.25)
	
robot.stop()
```

Looking at this in detail:

```python
import stretch_body.robot
```

The package stretch_body includes the Python module for Robot as well as other Devices such as Lift and Arm.

```python
robot=stretch_body.robot.Robot()
robot.startup()
```

Here we instantiate an instance of our Robot. The call to startup( ) opens the serial ports to the various devices, loads the Robot YAML parameters, and launches a few helper threads.

```python
for i in range(10):
	robot.pretty_print()
	time.sleep(0.25)
```

The call to pretty_print( ) prints to console all of the robot's sensor and state data. 

```python
robot.stop()
```

Finally, the stop( ) method shuts down the Robot threads and cleanly closes the open serial ports.

### Units

The Robot API uses SI units of:

* meters
* radians
* seconds
* Newtons
* Amps
* Volts

Parameters may be named with a suffix to help describe the unit type. For example:

* pos_m : meters
* pos_r: radians

### The Robot Status

The Robot derives from the [Device class](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/device.py). It also encapsulates a number of other Devices:

* [robot.head](.https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/head.py)
* [robot.arm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/arm.py)
* [robot.lift](.https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/lift.py)
* [robot.base](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/base.py)
* [robot.wacc](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wacc.py)
* [robot.pimu](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py)
* [robot.end_of_arm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py)

All devices contain a Status dictionary. The Status contains the most recent sensor and state data of that device. For example, looking at the Arm class we see:

```python
class Arm(Device):
    def __init__(self):
        ...
		self.status = {'pos': 0.0, 'vel': 0.0, 'force':0.0, \
                       'motor':self.motor.status,'timestamp_pc':0}
```

The Status dictionaries are  automatically updated by a background thread of the Robot at 25Hz. The Status data can be accessed via the Robot. For example:

```python
if robot.arm.status['pos']>0.25:
    print 'Arm extension greater than 0.25m'
```

If an instantaneous snapshot of the entire Robot Status is needed, the get_status() method can be used instead:

```python
status=robot.get_status()
if status['arm']['pos']>0.25:
    print 'Arm extension greater than 0.25m'
```

### The Robot Command

In contrast to the Robot Status which pulls data from the Devices, the Robot Command pushes data to the Devices.

Consider the following example which extends and then retracts the arm by 0.1 meters:

```python
import time
import stretch_body.robot

robot=stretch_body.robot.Robot()
robot.startup()

robot.arm.move_by(0.1)
robot.push_command()
time.sleep(2.0) 

robot.arm.move_by(-0.1)
robot.push_command()
time.sleep(2.0)
	
robot.stop()
```

A few important things are going on:

```
robot.arm.move_by(0.1)
```

The move_by( ) method queues up an RPC command to the stepper motor controller. However, the command does not yet execute.

```
robot.push_command()
```

The push_command( ) causes all queued up RPC commands to be executed at once.  In this example we call sleep( ) to allow time for the motion to complete before initiating a new motion.

**NOTE**: The Dynamixel servos do not use the Hello Robot RPC protocol. As such, the head, wrist, and gripper will move immediately upon issuing a motion command. 

The stepper actuators support a synchronous mode, allowing the base, arm, and lift to synchronously track trajectories.  Thus, the following code will cause the base, arm, and lift to initiate motion simultaneously:

```
robot.arm.move_by(0.1)
robot.lift.move_by(0.1)
robot.base.translate_by(0.1)
robot.push_command()
```

Commanding robot motion through the Stretch_Body interface is covered in more detail in the Robot Motion section.

### Stowing and Homing

After power up the robot requires homing in order for its joint encoders to find their zero position. The homing procedure will run the robot through a series of moves to find these zeros. It can be done programatically:

```python
if not robot.is_calibrated():
	robot.home() #blocking
```

Or it can be done manually after boot using the command line tool:

```bash
>>$ stretch_robot_home.py 
```

Likewise, stowing is a robot procedure that will cause it to move its arm and tool safely within the footprint of the base. 

```python
robot.stow() #blocking
```

Or it can be done manually from the command line when needed:

```bash
>>$ stretch_robot_stow.py 
```

## Scripting the Robot

A simplified design pattern to script the Robot is as follows

```python
#!/usr/bin/env python
import stretch_body.robot
from stretch_body.hello_utils import ThreadServiceExit

robot=stretch_body.robot.Robot()
robot.startup()

x_move_base = 0
x_move_arm = 0
x_move_lift = 0
x_move_head_pan = 0
x_move_head_tilt = 0
x_move_wrist_yaw = 0
x_move_gripper = 0

def update_my_behavior(status):
    #Update the joint commands based on the status data
    pass 

try:
	while True:
       	#Get a snapshot of the robot status data
        status=robot.get_status()
        
        #Compute new position targets based on sensor data 
        update_my_behavior(status)
        
        #Queue new targets to devices
        robot.base.translate_by(x_move_base) #or robot.base.rotate_by()
        robot.arm.move_by(x_move_arm)
        robot.lift.move_by(x_move_lift)
        robot.head.move_by('head_pan', x_move_head_pan)
        robot.head.move_by('head_tilt', x_move_head_tilt)
        robot.end_of_arm.move_by('wrist_yaw', x_move_wrist_yaw)
        robot.end_of_arm.move_by('stretch_gripper', x_move_gripper)
        
        #Synchronized send of new position targets 
        robot.push_command()
        
        #Wait for next control cycle
        time.sleep(0.1)
except (KeyboardInterrupt, SystemExit,ThreadServiceExit)
    pass

robot.stop()

```
## Command Line Tools

The Stretch_Body package comes with a suite of command line tools that allow direct interaction with hardware subsystems. These can be useful when developing and debugging applications. They also serve as code examples when developing applications for Stretch_Body.

These tools can be found by tab completion of  'stretch_' from a terminal.

```bash
>>$ stretch_


stretch_about.py                   
stretch_gripper_home.py            
stretch_lift_jog.py                
stretch_robot_dynamixel_reboot.py  
stretch_robot_stow.py              
stretch_wacc_scope.py              
stretch_arm_home.py                
stretch_gripper_jog.py             
stretch_pimu_jog.py                
stretch_robot_home.py              
stretch_robot_system_check.py      
stretch_wrist_yaw_home.py          
stretch_arm_jog.py                 
stretch_hardware_echo.py           
stretch_pimu_scope.py              
stretch_robot_jog.py               
stretch_rp_lidar_jog.py            
stretch_wrist_yaw_jog.py           
stretch_audio_test.py              
stretch_head_jog.py                
stretch_respeaker_test.py          
stretch_robot_keyboard_teleop.py   
stretch_urdf_show.py               
stretch_xbox_controller_teleop.py  
stretch_base_jog.py                
stretch_lift_home.py               
stretch_robot_battery_check.py     
stretch_robot_monitor.py           
stretch_wacc_jog.py 
```

All tools accept '--help' as a command line argument to learn its function. For example:

```bash
>>$ stretch_wacc_scope.py --help
usage: stretch_wacc_scope.py [-h] [--ax] [--ay] [--az] [--a0] [--d0] [--d1]
                             [--tap]

Visualize Wacc (Wrist+Accel) board data with an oscilloscope

optional arguments:
  -h, --help  show this help message and exit
  --ax        Scope accelerometer AX
  --ay        Scope accelerometer AY
  --az        Scope accelerometer AZ
  --a0        Scope analog-in-0
  --d0        Scope digital-in-0
  --d1        Scope digital-in-1
  --tap       Scope single tap

```

### Commonly Used Tools

These are the tools a typical user will want to become familiar with.

| **Tool**                              | **Utility**                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| **stretch_robot_home.py**             | Commonly run after booting up the robot in-order to calibrate the joints |
| **stretch_robot_system_check.py**     | Scans for all hardware devices and ensure they are present on the bus and reporting valid values. Useful to verify that the robot is in good working order prior to commanding motion. It will report all success in green, failures in red. |
| **stretch_robot_stow.py**             | Useful to return the robot arm and tool to a safe position within the base footprint. It can also be useful if a program fails to exit cleanly and the robot joints are not backdriveable. It will restore them to their 'Safety' state. |
| **stretch_robot_battery_check.py**    | Quick way to check the battery voltage / current consumption |
| **stretch_xbox_controller_teleop.py** | Useful to quickly test if a robot can achieve a task by manually teleoperating the robot |
| **stretch_robot_dynamixel_reboot.py** | This will reset all Dynamixels in the robot, which may be needed if a servo overheats during high use and enters an error state. |

The other tools are fairly self explanatory. They allow the user to quickly read a sensor value or control an individual hardware subsystem.

## Robot Motion

Controlling the motion of the robot's actuators is typically done through the ROS interfaces. However it is also possible to control the robot directly through the  stretch_body interfaces. 

Actuators are  commanded by either a **move_by** or **move_to** command (the former being incremental, the latter being absolute). For example, a relative move using the default motion parameters of the arm looks like:

```
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

robot.arm.move_by(0.1)
robot.push_command()
time.sleep(2.0)
	
robot.stop()
```

### Motion Profiles

All joints support trapezoidal based motion generation. Other types of controllers are available (PID, velocity, etc) but they are not covered here . The trapezoidal motion controllers require three values:

* x: target position of joint
* v: maximum velocity of motion
* a: acceleration of motion

We provide 'default' settings for the velocity and acceleration settings, as well as 'fast', and 'slow' settings. These values have been tuned to be appropriate for safe motion of the robot. These values can be seen in the 'stretch_re1_factory_params.yaml'. For example:

```yaml
arm:
  motion:
    fast: {accel_m: 0.2, vel_m: 0.2}
    default: {accel_m: 0.14, vel_m: 0.14}
    max: {accel_m: 1.0, vel_m: 1.0}
    slow: {accel_m: 0.07, vel_m: 0.06}
```

To move the arm quickly instead:

```python
vel_fast_m = robot.arm.params['motion']['fast']['vel_m']
accel_fast_m = robot.arm.params['motion']['fast']['accel_m']

robot.arm.move_by(x_m=0.1,v_m=vel_fast_m, a_m=accel_fast_m)
robot.push_command()

```

The motion will fall back to the 'default' settings found in the YAML if no parameters are provided.

### Range of Motion

All joints obey motion limits which are specified in the factory YAML. These limits have been set at the factory to prevent damage to the hardware. It is not recommended to set them to be greater than the factory specified values. However, they can be further limited if desired. 

For example. to prevent the lift from descending within 100mm of the base, one can override the factory setting in stretch_re1_user_params.yaml

```yaml
lift:
  range_m: [0.1, 1.095]
```

### Control Modes

Each joint has a default safety mode and default control mode.  These are:

| Joint           | Default Safety Mode         | Default Control Mode         |
| --------------- | --------------------------- | ---------------------------- |
| left_wheel      | Freewheel                   | Trapezoidal position control |
| right_wheel     | Freewheel                   | Trapezoidal position control |
| lift            | Gravity compensated 'float' | Trapezoidal position control |
| arm             | Freewheel                   | Trapezoidal position control |
| head_pan        | Torque disabled             | Trapezoidal position control |
| head_tilt       | Torque disabled             | Trapezoidal position control |
| wrist_yaw       | Torque disabled             | Trapezoidal position control |
| stretch_gripper | Torque disabled             | Trapezoidal position control |

The actuator remains in Safety Mode when no program is running. When the device.startup( ) function is called it transitions to its Control Mode. It is placed back in Safety Mode when device.stop() is called.

In addition, the Base supports a velocity control mode. The Base controllers will automatically switch between velocity and position based control.  For example:

```python
robot.base.translate_by(x_m=0.5)
robot.push_command()
time.sleep(4.0) #wait

robot.base.set_rotational_velocity(v_r=0.1) #switch to velocity controller
robot.push_command()
time.sleep(4.0) #wait

robot.base.set_rotational_velocity(v_r=0.0) #stop motion
robot.push_command()
```

As shown, care should be taken to reduce commanded velocities to zero to avoid runaway.

### Runstop

Runstop activation will cause  the Base, Arm, and Lift to switch to Safety Mode and for motion commands will be ignored. The motion commands will resume smoothly  when the runstop is deactivated.  This is usually done via the runstop button. However, it can also be done via the Pimu interface:

```python
if robot.pimu.status['runstop_event']:
	robot.pimu.runstop_event_reset()
    robot.push_command()
```

### Guarded Motion

The Arm, Lift, and Base support a guarded motion function.  It will automatically transition the actuator from Control mode to Safety mode when the exerted motor torque exceeds a threshold. 

This functionality is most useful for the Lift and the Arm. It allows these joints to safely stop upon contact. It can be used to:

* Safely stop when contacting an actuator hardstop
* Safely stop when making unexpected contact with the environment or a person
* Make a guarded motion where the robot reaches to a surface and then stops

Each of these tasks have different force characteristics and may require different threshold settings. The factory defaults are set so as to allow freespace motion without a payload (without triggering a false positive stop). These thresholds are set in the factory YAML. For example:

```yaml
arm:
  contact_thresh_N: [-50, 50]
  contact_thresh_max_N: [-80, 80]
```

A user can dynamically set the contact thresholds depending on the task requirements. For example, to make the arm extension motion more sensitive:

```python
robot.arm.move_by(x_m=0.1, contact_thresh_pos_N=30.0)
```

If too sensitive, the joint may trigger false positives (e.g., stop without contact), and different thresholds may be needed in different portions of the workspace.

When a guarded motion event has occurred it is reported in the Status:

```python
if robot.arm.motor.status['in_guarded_event']:
	print 'Arm has made contact'
```

The guarded event can be reset and motion resumed by simply sending a new motion command to the joint (that is not identical to the previous command). Here is a simple example of moving to contact, then moving back:

```python
robot.arm.move_to(0.5) #Reach all the way out
robot.push_command()

while robot.arm.status['pos']<0.5:
    if robot.arm.motor.status['in_guarded_event']:
        print 'Contact made at', robot.arm.status['pos']
        break
	time.sleep(0.1)
    
print 'Retracting...'
robot.arm.move_to(0.0)
robot.push_command( )

```


**Note: The units of Newtons are approximations only and may not be accurate to real world contact forces.**

### Synchronized Motion

The Arm, Lift, and Base actuators have a hardware synchronization mechanism. This allows for controller commands to be time synchronized across joints. By default these are turned out in the factory YAML:

```yaml
hello-motor-arm:
  gains: {enable_sync_mode: 1,...}
```

### Motion Status

It can be useful to poll the status of a joint during motion in order to modify the robot behavior, etc. The useful status values include:

```python
robot.arm.status['pos']						#Joint position
robot.arm.status['vel']						#Joint velocity
robot.arm.status['force']					#Joint force (derived from motor current)	
robot.arm.motor.status['near_pos_setpoint']	#Is sensed position near commanded position
robot.arm.motor.status['near_vel_setpoint'] #Is sensed velocity near commanded velocity
robot.arm.motor.status['is_moving']			#Is the joint in motion
robot.arm.motor.status['in_guarded_event']	#Has a guarded event occured
robot.arm.motor.status['in_safety_event']	#Has a safety event occured
```

### Update Rates

The following update rates apply to Stretch:

| Item                                            | Rate | Notes                                                        |
| ----------------------------------------------- | ---- | ------------------------------------------------------------ |
| Status data for Arm, Lift, Base, Wacc, and Pimu | 25Hz | Polled automatically by Robot thread                         |
| Status data for End of Arm and Head servos      | 15Hz | Polled automatically by Robot thread                         |
| Command data for Arm, Lift, Base, Wacc, Pimu    | N/A  | Commands are queued and executed upon calling robot.push_command( ) |
| Command data for End of Arm and Head servos     | N/A  | Commands execute immediately                                 |

Motion commands are non-blocking and it is the responsibility of the user code to poll the Robot Status to determine when and if a motion target has been achieved.

The Stretch_Body interface is not designed to support high bandwidth control applications. The natural dynamics of the robot actuators do not support high bandwidth contorl, and the USB based interface limits high rate communication.

In practice, a Python based control loop that calls push_command( ) at 1Hz to 10Hz is sufficiently matched to the robot natural dynamics. 

## Sensors

### Base IMU

Coming soon.

### Wrist Accelerometer

Coming soon.

### Cliff Sensors

Stretch has [four IR cliff sensors](https://docs.hello-robot.com/hardware_user_guide/#base) pointed towards the floor. These report the distance to the floor, allowing for detection of thresholds, stair edges, etc. 

Relevant parameters in the factory YAML are

```yaml
pimu:
  config:
    cliff_LPF: 10.0
    cliff_thresh: -50
    cliff_zero:
    - 523.7940936279297
    - 508.10246490478517
    - 496.55742706298827
    - 525.149652709961
    stop_at_cliff: 0
```

The  `stop_at_cliff` field causes the robot to execute a Runstop when the cliff sensor readings are out of bounds. 

**Note: As configured at the factory,  `stop_at_cliff` is set to zero and Stretch does not stop its motion based on the cliff sensor readings. Hello Robot makes no guarantees as to the reliability of Stretch's ability to avoid driving over ledges and stairs when this flag is enabled.**

The sensors are calibrated such that a zero value indicates the sensor is at the correct height from the floor surface. A negative value indicates a drop off such as a stair ledge while a positive value indicates an obstacle like a threshold or high pile carpet.

The calibrated range values from the sensors can be read from the `robot.pimu.status` message. Relevant fields are:

```python

In [1]: robot.pimu.pretty_print()
------ Pimu -----
...
At Cliff [False, False, False, False]
Cliff Range [2.043212890625, 3.710906982421875, 1.6026611328125, 1.95098876953125]
Cliff Event False
...
```

A Cliff Event flag is set when any of the four sensor readings exceed `cliff_thresh` and `stop_at_cliff` is enabled. In the event of a Cliff Event, it must be reset by `robot.pimu.cliff_event_reset()`in order to reset the generated Runstop.

The cliff detection logic can be found in the [Pimu firmware](https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_pimu/Pimu.cpp).

## Robot Parameters

All robot data is stored in the stretch_user directory.  The location of this directory can be found by:

```bash
>>$ echo $HELLO_FLEET_PATH
/home/hello-robot/stretch_user
```

The robot data stored here is identified by the robot ID (eg, stretch-re1-1002)

```bash
cd $HELLO_FLEET_PATH/$HELLO_FLEET_ID
>>$ ls
calibration_base_imu  
calibration_guarded_contact  
calibration_steppers    
calibration_D435i     
calibration_ros
export_urdf
udev
stretch_re1_factory_params.yaml  
stretch_re1_user_params.yaml
stretch_re1_tool_params.yaml
```

A factory image of this data (as shipped), is stored read-only under /etc/hello-robot . This is only for backup and to support cloning the user environment for new users.

### Calibration Data

The raw calibration data that was used in production for the robot is also stored for reference within the stretch_user directory. It isn't generally required for development.

### URDF Data

A calibrated URDF, and associated mesh files, are provided in the 'export_urdf' directory. This is provided for users who don't wish to use ROS yet still want an accurate model of the robot. The stretch_urdf_view.py tool demonstrates how to visualize the URDF from Python.

### YAML Data

Stretch_Body relies upon the following three primary YAML files:

| File                            | Purpose                                                      |
| ------------------------------- | ------------------------------------------------------------ |
| stretch_re1_factory_params.yaml | Factory settings for controller gains, calibrations, and system configuration. ***Read only *** |
| stretch_re1_user_params.yaml    | User parameters that override the factory parameters         |
| stretch_re1_tool_params.yaml    | Settings and configuration data for optional 3rd party end-of-arm tools. |

### Factory Parameters

This stretch_re1_factory_params.yaml file contains the robot's 'factory' settings. This includes things such as PID parameters for motor controllers, calibration constants, and default joint velocities and accelerations. 

The user should not edit this file. Hello Robot retains an 'as shipped' version of this file should it ever get corrupted. It can be instructive to review this file when getting to know the Stretch_Body code base.

### User Parameters

The factory settings should suffice for most use cases. However, the user is allowed to override the factory settings. This is done by using same YAML structure and name as is used in the stretch_re1_user_params.yaml file as in the factory file.

For example, heres the  stretch_re1_user_params.yaml  file is overriding the factory default contact thresholds and  motion speeds.

```yaml
factory_params: stretch_re1_factory_params.yaml
tool_params: stretch_re1_tool_params.yaml

lift:
  contact_thresh_N: [-60, 60]
  motion:
    default: {accel_m: 0.15, vel_m: 0.095}
arm:
  contact_thresh_N: [-80, 80]
  motion:
    default: {accel_m: 0.14, vel_m: 0.14}
base:
  motion:
    default: {accel_m: 0.1, vel_m: 0.15}


```

### End of Arm Tool Parameters

The stretch_re1_tool_params.yaml file stores configuration parameters specific to the user's custom end-of-arm-tools. It is read by the Robot class and the parameter data is made accessible to the user's end-of-arm-tool class. 

More information on integrating custom hardware on the End of Arm Dynamixel bus can be found at the [Extending Wrist DOF Tutorial](https://github.com/hello-robot/stretch_body/tree/master/tutorial/extending_wrist_dof)

## Safe Operation Features

Stretch includes a number of built-in functions that help it maintain safe operating conditions. These functions can be disabled and enabled via the robot YAML parameters.

### Logging

Upon instantiation, the Robot class opens a new log file for warning and informational messages to be written to. These timestamped logs are found under $HELLO_FLEET_DIRECTORY/log.

The logging messages can be echoed to the console by setting:

```
robot:
  log_to_console: 1
```

### Runstop Functions

| YAML                 | Function                                                |
| -------------------- | ------------------------------------------------------- |
| stop_at_low_voltage  | Trigger runstop / beep when voltage too low             |
| stop_at_high_current | Trigger runstop when bus current too high               |
| stop_at_cliff        | Trigger runstop when a cliff sensor is outside of range |
| stop_at_runstop      | Allow runstop to disable motors                         |
| stop_at_tilt         | Trigger runstop when robot tilts too far                |

### Robot Monitor

The [Robot Monitor](https://github.com/hello-robot/stretch_body/blob/master/python/stretch_body/robot_monitor.py) is a thread that monitors the Robot Status data for significant events. For example, it can monitor the error flags from the Dynamixel servos and notify when a thermal overload occurs. The Robot Monitor logs warnings to a log file by default. 

| YAML                     | Function                                                     |
| ------------------------ | ------------------------------------------------------------ |
| monitor_base_bump_event  | Report when the accelerometer detects a bump event           |
| monitor_base_cliff_event | Report when a cliff sensor event occurs                      |
| monitor_current          | Report when the battery current exceeds desired range        |
| monitor_dynamixel_flags  | Report when a Dynamixel servo enters an error state          |
| monitor_guarded_contact  | Report when a guarded contact event occurs                   |
| monitor_over_tilt_alert  | Report when an over-tilt event occurs                        |
| monitor_runstop          | Report when the runstop is activated / deactivated           |
| monitor_voltage          | Report when the battery voltage is out of range              |
| monitor_wrist_single_tap | Report when the wrist accelerometer reports a single tap event |

The YAML below illustrates the types of events that are can be configured.

```yaml
robot:
  log_to_console: 0
  use_monitor: 1
  use_sentry: 1
  
robot_monitor:
  monitor_base_bump_event: 1
  monitor_base_cliff_event: 1
  monitor_current: 1
  monitor_dynamixel_flags: 1
  monitor_guarded_contact: 1
  monitor_over_tilt_alert: 1
  monitor_runstop: 1
  monitor_voltage: 1
  monitor_wrist_single_tap: 1

robot_sentry:
  base_fan_control: 1
  base_max_velocity: 1
  stretch_gripper_overload: 1
  wrist_yaw_overload: 1
```

### Robot Sentry

The [Robot Sentry](https://github.com/hello-robot/stretch_body/blob/master/python/stretch_body/robot_sentry.py) is a thread that can override and also generate commands to the robot hardware. It's purpose is to keep the robot operating within a safe regime. For example, the Robot Sentry monitors the position of the Lift and Arm and limits the maximum base velocity and acceleration (in order to reduce the chance of toppling). The Robot Sentry reports events to the log file as well. 

| YAML                     | Function                                                     |
| ------------------------ | ------------------------------------------------------------ |
| base_fan_control         | Turn the fan on when CPU temp exceeds range                  |
| base_max_velocity        | Limit the base velocity when robot CG is high                |
| stretch_gripper_overload | Reset commanded position to prevent thermal overload during grasp |
| wrist_yaw_overload       | Reset commanded position to prevent thermal overload during pushing |



------
.<div align="center"> All materials are Copyright 2020 by Hello Robot Inc. The Stretch RE1 robot has patents pending</div>