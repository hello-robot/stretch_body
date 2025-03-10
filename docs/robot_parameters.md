# Robot Parameters

The behavior of Stretch's hardware and software is tweakable through "robot parameters", which are dozens of key-value pairs stored in YAML files and read into Stretch Body when the Python SDK is used. To learn about the parameter system and tweaking the values, follow the [Parameter Management Tutorial](../../stretch_tutorials/stretch_body/tutorial_parameter_management.md). In this document, a description of every parameter and its default value is provided.

### use_asyncio

A boolean to toggle the use of Asyncio for coordination serial communication with the stepper motors. Using asyncio enables the program flow to move onto other computation while waiting for a reply from a stepper.

| Parameter         | Default Value |
|-------------------|---------------|
| robot.use_asyncio | `1`           |

### use_collision_manager

A boolean to toggle the use of the collision manager, which prevents self-collisions between the robot's arm and body. For example, this can be helpful for novice users during gamepad teleop because they can figure out the controls without fear of teleoperating the robot into collisions.

| Parameter                   | Default Value |
|-----------------------------|---------------|
| robot.use_collision_manager | `0`           |

### params

Additional sources of parameters for Stretch Body to import in when organizing the robot's complete set of parameters. This parameter is an array of strings, where each string is an importable Python module. Therefore, it's important that your additional source of parameters is on the "Python Path" (i.e. you can import it from Python).

The most common reason to set an additional parameter source is to supply parameters relating to a new [tool](#tool).

| Parameter | Default Value |
|-----------|---------------|
| params    | `[]`          |

### tool

You can swap the tool (or "end-effector") on Stretch. There's a [repository of 3D printable tools](https://github.com/hello-robot/stretch_tool_share/blob/master/README.md) that includes a USB camera tool, a phone holder tool, a dry erase marker tool, and many more. Many of these tools come with a Python class that extends Stretch Body's `EndOfArm` class to implement functionality specific to the tool. For example, the USB camera tool comes with a [`ToolUSBCamWrist`](https://github.com/hello-robot/stretch_tool_share/blob/master/python/stretch_tool_share/usbcam_wrist_v1/tool.py) class, named `tool_usbcam_wrist`, to take pictures or return a video stream directly from Stretch Body. You can also create your own custom tools and extend `EndOfArm` to expose functionality custom to your tool. More information on swapping/creating tools is available in the [Tool Change Tutorial](../../stretch_tutorials/stretch_body/tutorial_tool_change.md).

After attaching the hardware to Stretch's wrist, set this parameter to your tool class's name to change which tool class is imported into Stretch Body. See also [params](#params) for supplying parameters needed by your tool class.

| Parameter  | Default Value            |
|------------|--------------------------|
| robot.tool | `'tool_stretch_gripper'` |

### use_multiturn

The Dynamixel joints on the robot have a "multiturn" or "Extended Position Control" mode, which allows the Dynamixel servo to rotate many revolutions. This is in contrast to regular "Position Control" mode, where the servo is limited to 360° rotation. The joints where the servos have gear reductions operate in multiturn mode to achieve the desired range. In the other joints, the servo directly controls the position of the joint, so multiple revolutions are not required. More details on the Dynamixel's control modes are [available here](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#operating-mode11).

| Parameter                     | Default Value |
|-------------------------------|---------------|
| head_pan.use_multiturn        | `0` *         |
| head_tilt.use_multiturn       | `0`           |
| stretch_gripper.use_multiturn | `1`           |
| wrist_yaw.use_multiturn       | `1`           |
| wrist_pitch.use_multiturn     | `0`           |
| wrist_roll.use_multiturn      | `0`           |

\* `head_pan.use_multiturn` is `0` for most Stretch robots, except for some early RE1s. For those robots, the parameter is set to `1` in "stretch_configuration_params.yaml". 

### enable_runstop

This parameter changes how a particular joint behaves when the robot is runstopped. Setting this parameter to `0` means the joint will not stop moving and continue tracking its commanded goal. This can be useful if, for example, you want the gripper to keep hold on what it's holding when the robot is runstopped.

The Dynamixel joints on the robot handle runstop differently than the stepper joints do. Whereas the stepper joints receive a hardware pulse whenever the runstop button is pressed, the Dynamixel joints don't. When Stretch Body is running, a sentry monitors the runstop through the pimu and calls `disable_torque()` / `enable_torque()` on the dxl joints as needed. For stepper joints, this parameter gives you control over whether the stepper PCB respects the hardware pulse. For dxl joints, this parameter gives you control over whether the sentry will turn on/off torque of a joint in response to the runstop state.

| Parameter                                    | Default Value |
|----------------------------------------------|---------------|
| head_pan.enable_runstop                      | `1`           |
| head_tilt.enable_runstop                     | `1`           |
| stretch_gripper.enable_runstop               | `1`           |
| wrist_yaw.enable_runstop                     | `1`           |
| wrist_pitch.enable_runstop                   | `1`           |
| wrist_roll.enable_runstop                    | `1`           |
| hello-motor-arm.gains.enable_runstop         | `1`           |
| hello-motor-lift.gains.enable_runstop        | `1`           |
| hello-motor-left-wheel.gains.enable_runstop  | `1`           |
| hello-motor-right-wheel.gains.enable_runstop | `1`           |

To disable the sentry, there's also the `robot_sentry.dynamixel_stop_on_runstop` parameter. To disable the hardware pulses, there's also the `pimu.config.stop_at_runstop` parameter. To disabling logging when runstop is enabled/disabled, there's also the `robot_monitor.monitor_runstop`.

### i_feedforward and i_safety_feedforward

Gravity compensation adds a fixed ‘feedforward’ current to the motor controller to support the lift against gravity. This allows the lift to ‘float’ when the runstop is enabled, for example. If the feedforward current is too low, the lift will drift downward. If it is too high, it will drift upward. The `i_safety_feedforward` is the amount of current (A) applied when the motor is in safety mode (eg, runstop enabled). The `i_feedforward` term is applied when the lift is in normal operation. Generally the two parameters will be identical.

There’s a simple tool to calibrate these values. `REx_calibrate_gravity_comp.py` will move the lift to a few positions, sampled the applied motor currents, and update your gravity compensation parameters. More details can be found in this [knowledge base post](https://forum.hello-robot.com/t/practical-guide-to-lift-gravity-compensation/657).

### stretch_gripper_overload

A boolean to toggle whether a sentry monitors the gripper servo for risk of "overloading", a hardware protection state the servo goes into when it cannot provide the torque being asked for, and backs off the commands to reduce the amount of torque being asked for. In effect, this sentry enables the gripper to keep its grip on objects without overloading.

| Parameter                             | Default Value |
|---------------------------------------|---------------|
| robot_sentry.stretch_gripper_overload | `1`           |

### base_max_velocity

A boolean to toggle whether a sentry monitors the robot's center of mass and limits max allowable speed of the mobile base to prevent unstable behavior resulting from fast motion paired with a high center of mass. Disabling this safety feature means the base will not limit its speed and will travel at the speed you've commanded it.

| Parameter                      | Default Value |
|--------------------------------|---------------|
| robot_sentry.base_max_velocity | `1`           |

### motion

A set of defaults for velocity and acceleration presets. The presets include "default", "slow", "fast", "max", and "trajectory_max". All move_by/move_to commands use "default" by default. These presets are available via `<joint>.params['motion']['<preset>']`, so e.g. to move the lift slowly, you could use:

```python
robot.lift.move_to(1.1, v_m=robot.lift.params['motion']['slow']['vel_m'], a_m=robot.lift.params['motion']['slow']['accel_m'])
```

| Parameter                    | Default Value |
|------------------------------|---------------|
| head_pan.motion.default.vel        | `3.0`        |
| head_pan.motion.default.accel        | `8.0`        |
| head_tilt.motion.default.vel        | `3.0`        |
| head_tilt.motion.default.accel        | `8.0`        |
| stretch_gripper.motion.default.vel        | `6.0`        |
| stretch_gripper.motion.default.accel        | `10.0`        |
| wrist_yaw.motion.default.vel        | `2.0`        |
| wrist_yaw.motion.default.accel        | `3.0`        |
| wrist_pitch.motion.default.vel        | `2.0`        |
| wrist_pitch.motion.default.accel        | `6.0`        |
| wrist_roll.motion.default.vel        | `2.0`        |
| wrist_roll.motion.default.accel        | `8.0`        |
| arm.motion.default.vel_m        | `0.14`        |
| arm.motion.default.accel_m        | `0.14`        |
| lift.motion.default.vel_m        | `0.11`        |
| lift.motion.default.accel_m        | `0.2`        |
| base.motion.default.vel_m        | `0.12`        |
| lift.motion.default.accel_m        | `0.12`        |

Units are m/s and m/s^2 for prismatic joints, and rad/s and rad/s^2 for rotary joints. To find the values of the other presets, use `stretch_params.py | grep <joint>.motion.<preset>` in the terminal.

### distance_tol

A tolerance within which the first waypoint of a splined trajectory must be of the joint's current position when `follow_trajectory(move_to_first_point=False)`, or the trajectory won't start execution.

| Parameter                    | Default Value |
|------------------------------|---------------|
| head_pan.distance_tol        | `0.15`        |
| head_tilt.distance_tol       | `0.52`        |
| stretch_gripper.distance_tol | `0.015`       |
| wrist_yaw.distance_tol       | `0.015`       |
| wrist_pitch.distance_tol     | `0.03`        |
| wrist_roll.distance_tol      | `0.015`       |
| arm.distance_tol             | `0.008`       |
| lift.distance_tol            | `0.015`       |

## Parameters for Command Line Tools

### show_sw_exc

In the `stretch_robot_system_check.py` tool, the "Checking Software" section is wrapped in a try/except because of the experimental nature of the code. The tool is introspecting the system and relying on resources/files to be available in specific places with specific formats. Since the tool's output needs to be easily understood, the error or exception cannot be shown (hence the try/except block). This parameter allows the tool to raise the exception and display it in the terminal. This is useful for debugging purposes.

| Parameter                | Default Value |
|--------------------------|---------------|
| system_check.show_sw_exc | False         |
