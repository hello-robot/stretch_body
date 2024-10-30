# Robot Parameters

The behavior of Stretch's hardware and software is tweakable through "robot parameters", which are dozens of key-value pairs stored in YAML files and read into Stretch Body when the Python SDK is used. To learn about the parameter system and tweaking the values, follow the [Parameter Management Tutorial](../../stretch_tutorials/stretch_body/tutorial_parameter_management.md). In this document, a description of every parameter and its default value is provided.

### use_asyncio

A boolean to toggle the use of Asyncio for coordination serial communication with the stepper motors. Using asyncio enables the program flow to move onto other computation while waiting for a reply from a stepper.

| Parameter         | Default Value |
|-------------------|---------------|
| robot.use_asyncio | `1`           |

### use_collision_manager

A boolean to toggle the use of the collision manager, which prevents self-collisions between the robot's arm and body. For example, this can be helpful for novice users during gamepad teleop because they can figure out the controls without fear of teleoperating the robot into collisions.

| Parameter         | Default Value |
|-------------------|---------------|
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

### i_feedforward and i_safety_feedforward

Gravity compensation adds a fixed ‘feedforward’ current to the motor controller to support the lift against gravity. This allows the lift to ‘float’ when the runstop is enabled, for example. If the feedforward current is too low, the lift will drift downward. If it is too high, it will drift upward. The `i_safety_feedforward` is the amount of current (A) applied when the motor is in safety mode (eg, runstop enabled). The `i_feedforward` term is applied when the lift is in normal operation. Generally the two parameters will be identical.

There’s a simple tool to calibrate these values. `REx_calibrate_gravity_comp.py` will move the lift to a few positions, sampled the applied motor currents, and update your gravity compensation parameters. More details can be found in this [knowledge base post](https://forum.hello-robot.com/t/practical-guide-to-lift-gravity-compensation/657).

## Parameters for Command Line Tools

### show_sw_exc

In the `stretch_robot_system_check.py` tool, the "Checking Software" section is wrapped in a try/except because of the experimental nature of the code. The tool is introspecting the system and relying on resources/files to be available in specific places with specific formats. Since the tool's output needs to be easily understood, the error or exception cannot be shown (hence the try/except block). This parameter allows the tool to raise the exception and display it in the terminal. This is useful for debugging purposes.

| Parameter                | Default Value |
|--------------------------|---------------|
| system_check.show_sw_exc | False         |
