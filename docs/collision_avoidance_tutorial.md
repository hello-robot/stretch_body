![](./images/banner.png)
# Working with the Collision Avoidance System

In this tutorial we will discuss the simple collision avoidance system that runs as a part of Stretch Body.

**Note**: This tutorial applies to Stretch Body version 0.1.x or greater.

## Overview

Stretch Body includes a system to prevent inadvertent self-collisions.  

**Note**: Self collisions are still possible while using the collision-avoidance system. The factory default collision models are coarse and not necessarily complete.

## Joint Limits

The collision avoidance system works by dynamically modifying the acceptable range of motion for each joint. By default, a joint's range is set to the physical hardstop limits. For example, the lift has a mechanical throw of 1.1m:

```bash
>>$stretch_params.py | grep range | grep lift
stretch_body.robot_params.factory_params       param.lift.range_m      [0.0, 1.1]                
```

A reduced range-of-motion can be set at run-time by setting the Soft Motion Limit. For example, to limit the lift range of motion to 0.3 meter off the base:

```python
import stretch_body.robot as robot
r=robot.Robot()
r.startup()
r.lift.set_soft_motion_limit_min(0.3)
```

We see in the [API](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/lift.py), the value of `None` is used to designated no soft limit.

It is possible that when setting the Soft Motion Limit that the joints current position is outside of the specified range. In this case, the joint will move to the nearest soft limit so as to comply with the limits. This can be demonstrated by:

```python
import stretch_body.robot as robot
import time

r=robot.Robot()
r.startup()

#Move to 0.2
r.lift.move_to(0.2)
r.push_command()
time.sleep(5.0) 

#Will move to 0.3
r.lift.set_soft_motion_limit_min(0.3)

```



## Collision Models

The [RobotCollision](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) class manages a set of [RobotCollisionModels](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py). Each [RobotCollisionModel](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) computes the soft limits for a subset of joints based on a simple geometric model. 

The `step` method of a RobotCollisionModel returns the desired joint limits given that model. For example, the base class is simply:

```python
    class RobotCollisionModel(Device):
    	def step(self, status):
        	return {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, None],'arm': [None, None],'wrist_yaw': [None, None]}
```

, where the value of `None` specifies that no-limit is specified and the full range-of-motion for the joint is acceptable.  

We could define a new collision model that simply limits the lift range of motion to 1 meter by:

```python
    class MyCollisionModel(Device):
    	def step(self, status):
        	return {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, 1.0],'arm': [None, None],'wrist_yaw': [None, None]}
```

Each model is registered with the [RobotCollision](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) instance as a loadable plug-in. The [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) class calls the `RobotCollision.step` method periodically at approximately 10hz. 

`RobotCollision.step`  computes the 'AND' of the  limits specified across each Collision Model such that the most restrictive joint limits are set for each joint using the `set_soft_motion_limit_min , set_soft_motion_limt_max` methods. 

## Default Collision Models

The default collision models for Stretch Body are found in [robot_collision_models.py](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision_models.py). As of this writing, the provide models are:

* CollisionArmCamera: Avoid collision of the head camera with the arm
* CollisionStretchGripper: Avoid collision of the wrist-yaw and gripper with the base and ground

**Note**: The provided collision models are coarse and are provided to avoid common potentially harmful collisions only. Using these models it is still possible to collide the robot with itself in some cases.

### Working with Models

The collision models to be used by Stretch Body are defined with the `robot_collision` parameter. For example, we see in `robot_params.py`  that the CollisionArmCamera is loaded by default

```python
"robot_collision": {'models': ['collision_arm_camera']},
```

We also see that model `collision_arm_camera` is defined as:

```python
  "collision_arm_camera": {
        'enabled': 1,
        'py_class_name': 'CollisionArmCamera',
        'py_module_name': 'stretch_body.robot_collision_models'
    }
```

This instructs RobotCollision to construct a model of type `CollisionArmCamera` and enable it by default. One can disable this model by default by specifying the following `stretch_re1_user_params.yaml`:

```yaml
collision_arm_camera:
  enabled: 0
```

The  entire collision avoidance system can be disabled in `stretch_re1_user_params.yaml` by:

```yaml
robot:
  use_collision_manager: 0
```

A specific collision model can be enabled or disabled during runtime by:

```python
import stretch_body.robot
r=stretch_body.robot.Robot()
r.startup() 
... #Do some work
r.collision.disable_model('collsion_arm_camera')
... #Do some work
r.collision.enable_model('collsion_arm_camera')
```

Finally, if we want to also use the CollisionStretchGripper model, we can add to `stretch_re1_user_params.py`:

```yaml
robot_collision:
  models:
  - collision_arm_camera
  - collision_stretch_gripper
```

### Creating a Custom Collision Model

It can be straightforward to create your own custom collision model. As an example, we will create a model that avoids collision of the arm with a table top by

* Prevent the lift from descending below the table top when the arm is extended 
* Allow the lift to descend below the tabletop so long as the arm retracted

This assumes the arm is initially above the table top. To start, in a file `collision_arm_table.py` we add:

```python
from stretch_body.robot_collision import *
from stretch_body.hello_utils import *

class CollisionArmTable(RobotCollisionModel):
    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_arm_table')

    def step(self, status):
        limits = {'lift': [None, None],'arm': [None, None]}
        table_height = 0.5 #m
        arm_safe_retract = 0.1 #m
        safety_margin=.05#m

        x_arm = status['arm']['pos']
        x_lift = status['lift']['pos']

        #Force arm to stay retracted if below table
        if x_lift<table_height:
            limits['arm'] = [None, arm_safe_retract-safety_margin]
        else:
            limits['arm'] = [None, None]

        #Force lift to stay above table unless arm is retracted
        if x_arm<arm_safe_retract:
            limits['lift'] =[None,None]
        else:
            limits['lift']=[table_height+safety_margin,None]
        return limits
```
In this example we include the `safety_margin` as a way to introduce some hysteresis around state changes to avoid toggling between the soft limits.

The following command should be run in order to add the working directory to the PYTHONPATH env , This can also be added to our bashrc to permanently edit the path: 

```bash
>>$ export PYTHONPATH=$PYTHONPATH:/<path_to_modules>
```


Next we configure RobotCollision to use our CollisionArmTable model in `stretch_re1_user_yaml`:

```yaml
robot_collision:
  models:
  - collision_arm_table

collision_arm_table:
  enabled: 1
  py_class_name: 'CollisionArmTable'
  py_module_name: 'collision_arm_table'

```

Finally, test out the model by driving the arm and lift around using the XBox teleoperation tool:

```bash
>>$ stretch_xbox_controller_teleop.py
```

