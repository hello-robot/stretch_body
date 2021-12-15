![](../images/banner.png)

# Extending the Wrist DOF

In this tutorial we explore how to add additional degrees of freedom to the Stretch wrist. 

Stretch exposes a Dynamixel X-Series TTL control bus at the end of its arm. It uses the [Dynamixel XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) for the [WristYaw](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wrist_yaw.py) and the [StretchGripper](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/stretch_gripper.py)  degrees of freedom that come standard with the robot. 

See the [Hardware User Guide](https://docs.hello-robot.com/hardware_user_guide/#wrist) to learn how to mechanically attach additional DOFs to the robot.

**Note: Stretch is compatible with [any Dynamixel X Series servo](https://emanual.robotis.com/docs/en/dxl/x/) that utilizes the TTL level Multidrop Bus.**

## Adding a Custom DOF

Adding one or more custom Dynamixel X Series servos to Stretch wrist involves:

* Creating a new class that derives from [DynamixelHelloXL430](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_hello_XL430.py)
* Adding YAML parameters to `stretch_re1_tool_params.yaml `that configure the servo as desired
* Adding YAML parameters to `stretch_re1_user_params.yaml` that tell Stretch to include this class in its EndOfArm list of servos

Let's create a new DOF called WristPitch in a file named [wrist_pitch.py](./wrist_pitch.py).  Place the file somewhere on the $PYTHONPATH.

```python
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *

class WristPitch(DynamixelHelloXL430):
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'wrist_pitch', chain)
        self.poses = {'tool_up': deg_to_rad(45),
                      'tool_down': deg_to_rad(-45)}

    def pose(self,p,v_r=None,a_r=None):
        self.move_to(self.poses[p],v_r,a_r)
```

Now let's copy in [YAML parameters for your servo](./stretch_re1_tool_params.yaml) to your `stretch_re1_tool_params.yaml` in order to configure this servo. You may want to adapt these parameters to your application but the nominal values shown usually work well. Below we highlight some of the more useful parameters.

```yaml
wrist_pitch:
  id: 1							#ID on the Dynamixel bus
  range_t:						#Range of servo, in ticks
  - 0
  - 4096
  req_calibration: 0			#Does the joint require homing after startup
  use_multiturn: 0				#Single turn or multi-turn mode of rotation
  zero_t: 2048					#Position in ticks that corresponds to zero radians
```

For this example we are assuming a single turn joint that doesn't require hardstop based homing. We also assume the servo has the Robotis default ID of 1.

At this point your WristPitch class is ready to use. Plug the servo into the cable leaving the Stretch WristYaw joint. Experiment with the API from iPython

```python
Python 2.7.17 (default, Apr 15 2020, 17:20:14) 
...

In [1]: import wrist_pitch

In [2]: w=wrist_pitch.WristPitch()

In [3]: w.startup()

In [4]: w.move_by(0.1)

In [5]: w.pose('tool_up')

In [6]: w.pose('tool_down')
```

Finally, you'll want to make your WristPitch available from `stretch_body.robot` Add the following [YAML](./stretch_re1_user_params.yaml) to your `stretch_re1_user_params.yaml`

```yaml
end_of_arm:
  devices:
    wrist_pitch:
      py_class_name: WristPitch
      py_module_name: wrist_pitch
```

This tells `stretch_body.robot` to manage a wrist_`pitch.WristPitch`instance and add it to the [EndOfArm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py) list of tools. Try it from iPython:

```python
Python 2.7.17 (default, Jul 20 2020, 15:37:01)
...
In [1]: import stretch_body.robot as robot

In [2]: r=robot.Robot()

In [3]: r.startup()

In [4]: r.end_of_arm.move_by('wrist_pitch',0.1)

```



## Additional Information

### The [DynamixelHelloXL430](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_hello_XL430.py) Class

The DynamixelHelloXL430 class is the primary interface to the Dynamixel servos of the robot. While named 'XL430' for legacy reasons, this class should be compatible with other X Series servos so long as they have compatible Control Tables (see the Dynamixel documentation).

The role of DynamixelHelloXL430 is to provide a calibrated interface to the servo in radians, manage homing to hardstops, and to expose an interface to `stretch_body` that is consistent with the robot's other DOF. 

A class, such as [WristYaw](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wrist_yaw.py), derives from DynamixelHelloXL430, and extends it to include functionality that is specific to that DOF. For example, we can see in  [WristYaw](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wrist_yaw.py) that it provides an interface to move to predetermined poses.  

### The [DynamixelXChain](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_X_chain.py) Class and [EndOfArm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py) Class

The DynamixelXChain class manages a set of DynamixelHelloXL430 devices for `stretch_body`. It also provides synchronized read access to the servos in order to reduce  bus communication bottlenecks

The EndOfArm class is derived from DynamixelXChain. It includes functionality that allows it to read the YAML and instantiate custom DynamixelHelloXL430 devices as shown in the example above.

### Setting the Servo ID

By default the Dynamixel servo has an ID of 1. Each servo on a bus must have a unique ID. You can scan the bus for IDs using the `RE1_dynamixel_id_scan.py` tool:

```bash
>>$ RE1_dynamixel_id_scan.py /dev/hello-dynamixel-wrist 
[Dynamixel ID:000] ping Failed.
[Dynamixel ID:001] ping Failed.
[Dynamixel ID:002] ping Failed.
[Dynamixel ID:003] ping Succeeded. Dynamixel model number : 1060
[Dynamixel ID:004] ping Succeeded. Dynamixel model number : 1020
[Dynamixel ID:005] ping Succeeded. Dynamixel model number : 1020
[Dynamixel ID:006] ping Succeeded. Dynamixel model number : 1060
...

```

Here we see that devices with IDs 3-6 are on the bus. To change the ID for device 6 to 7 for example, use `RE1_dynamixel_id_change.py`

```bash
>>$RE1_dynamixel_id_change.py /dev/hello-dynamixel-wrist 6 7
[Dynamixel ID:006] ping Succeeded. Dynamixel model number : 1060
Ready to change ID to 7 . Hit enter to continue

[Dynamixel ID:007] ping Succeeded. Dynamixel model number : 1060
Success at setting ID to 7
```



### Homing a Multi-Turn Servo

By default the Dynamixel servos are configured to be single-turn devices. When in single turn mode, they do not require a homing procedure on startup. Multi-turn devices, such as the Stretch WristYaw and StretchGripper, require a homing procedure.

The homing procedure moves the joint to one or both mechanical limits of the joint and, based on the detected hardstops, sets the joint's 'zero' point. 

Your custom DynamixelHelloXL430 device may use this functionality if desired. To do so, you'll modify the following YAML fields for the device:

```yaml
wrist_pitch:
  ...
  pwm_homing: #Set the force that the joint approaches the hardstop
  - -300
  - 300
  range_t: #Set the mechanical range of the joint in ticks
  - 0
  - 9340
  req_calibration: 1 #Don't allow the joint to move until it has been homed
  use_multiturn: 1 #Enable multi-turn mode on the servo
  zero_t: 7175 #Mark the position in the joint range to call zero
```

As shown above, the robot expects a mechanical range of motion of 9340 ticks (as defined in the Dynamixel documentation). Once homed, the DynamixelHelloXL430 device will report `pos=0` radians when the servo is at tick 7175. 

In order to home the joint you can simply:

```python
import wrist_pitch
w=wrist_pitch.WristPitch()
w.startup()
w.home(single_stop=True, move_to_zero=True)
```

This will cause it to move to just the first stop (at -300 PWM), and the move to zero (eg tick 7175) when done.

Finally, if your custom device is registered with EndOfArm, it will automatically home along with the other joints when calling `stretch_robot_home.py`.





