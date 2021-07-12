![](./images/banner.png)
# Working With Collision Avoidance

In this tutorial we will discuss the simple collision avoidance system that runs as a part of Stretch Body.

**Note**: This tutorial applies to Stretch Body version 0.1.x or greater.

## Overview

Stretch Body includes a system to prevent inadvertent self-collisions. The [RobotCollision](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) class manages a set of [RobotCollisionModels](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py). Each [RobotCollisionModel](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) computes the soft limits for a joint's range of motion based on a simple geometric model. 

This avoidance system is coarse by design and serves the purpose to avoid common potentially harmful collisions only.

In the base class of RobotCollisionModel we see the simplest `null` model:

```python
    class RobotCollisionModel(Device):
    	def step(self, status):
        	return {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, None],'arm': [None, None],'wrist_yaw': [None, None]}
```

The value of `None` specifies that no-limit is specified and the full range-of-motion for the joint is acceptable.  We could define a new collision model that simply limits the lift range of motion to 1 meter by:

```python
    class MyCollisionModel(Device):
    	def step(self, status):
        	return {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, 1.0],'arm': [None, None],'wrist_yaw': [None, None]}
```

Each model is registered with the [RobotCollision](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) instance as a loadable plug-in. The [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) class manages the RobotCollision instance, which calls the `RobotCollision.step` method periodically at approximately 10hz. 

RobotCollision computes the 'AND' of the  limits specified across each RobotCollisionModel such that the most restrictive joint limits are use. It then commands the new joint limits to the joint using the `set_soft_motion_limits` method. For example, in `RobotCollision.step` we see:

```python
 self.robot.lift.set_soft_motion_limits(limits['lift'][0], limits['lift'][1])
```

### Provided Collision Models

The provided collision models for Stretch Body are found in [robot_collision_models.py](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision_models.py). As of this writing, the provide models are:

* CollisionArmCamera: Avoid collision of the head camera with the arm
* CollisionStretchGripper: Avoid collision of the wrist-yaw and gripper with the base and ground

**Note**: The provided collision models are very coarse and are provided to avoid common potentially harmful collisions only.

### Working with Models

The RobotCollision instance will load a model based on the  `robot_collision` parameter. We see in `robot_params.py`  that by default only the CollisionArmCamera is loaded by default

```python
"robot_collision": {'models': ['collision_arm_camera']},
```

In turn, in the same file we see that model `collision_arm_camera` is defined as:

```python
  "collision_arm_camera": {
        'enabled': 1,
        'head_pan_avoid_clip_neg': -0.51,
        'head_pan_avoid_clip_pos': 0.38,
        'head_tilt_avoid_clip': -1.4,
        'lift_approach_clip': 0.95,
        'lift_near_clip': 1.02,
        'py_class_name': 'CollisionArmCamera',
        'py_module_name': 'stretch_body.robot_collision_models'
    }
```

This instructs RobotCollision to construct a model of type `CollisionArmCamera` and enable it by default. One can disable this model by default by specifying the following `stretch_re1_user_params.yaml`:

```yaml
collision_arm_camera:
  enabled: 0
```

The entire collision avoidance system can be disabled if desired by adding the following to `stretch_re1_user_params.yaml`:

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

Finally, to include the model CollisionStretchGripper in the system, we can add to `stretch_re1_user_params.py`:

```yaml
robot_collision:
  models:
  - collision_arm_camera
  - collision_stretch_gripper
```

### Creating a Custom Collision Model

It can be straightforward to create your own custom collision model. As an example, we will create a model that avoids collision of the arm with a table top when the arm is extended (beyond 0.2m) and the lift is descending (below 0.3m):

To start, in our file `collision_arm_table.py` we add

```python
from stretch_body.robot_collision import *
import math
from stretch_body.hello_utils import *

class CollisionArmTable(RobotCollisionModel):

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_arm_table')

    def step(self, status):
        limits = {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, None],'arm': [None, None],'wrist_yaw': [None, None]}
        x_arm = status['arm']['pos']
        x_lift = status['lift']['pos']
        if x_arm>0.2 and x_lift>0.3:
            limits['lift']=[0.3,None]
        
```

First, let's define a class that derives from RobotCollisionModel and configure it in `stretch_re1_user_yaml`:

