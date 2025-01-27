# Changelog

The changes between releases of Stretch Body are documented here.

## [0.7.28](https://pypi.org/project/hello-robot-stretch-body/0.7.13/) - January 27, 2025
 - Drops Numba as a dependency

## [0.7.13](https://pypi.org/project/hello-robot-stretch-body/0.7.13/) - May 7, 2024
 - This PR robot process protection to multi-user setup. Currently, if you have an ongoing connection to the robot, and attempt to launch another application that needs to connect to the USB hardware, the second attempt will not connect and will let the user know that an ongoing connection already exists. This is because only one connection to the hardware can safely communicate at a time. Therefore, this protects the first connection from experiencing dropouts or interruptions. The warning also informs the user that they can kill the first connection using the `stretch_free_robot_process.py` CLI.
 - All of the above protections work well on single user set-ups, which is the default Stretch ships with. However, it's common in developer teams that are sharing the robot to want to create multiple Unix users on the robot. This PR enables all of the same protections and CLIs that worked previously to now work with multi-user setups.

## [0.7.11](https://pypi.org/project/hello-robot-stretch-body/0.7.11/) - Feburary 20, 2024
 - Introduce the `stretch_configure_tool.py` tool (pr [#287](https://github.com/hello-robot/stretch_body/pull/287))
    - This PR introduces a new CLI called stretch_configure_tool.py. It eliminates a pain point around changing the robot.tool parameter and updating the URDF (and exported URDF).
    - This tool supports all three models, all officially supported tools, Ubuntu 20.04 and 22.04.
 - Add "Tool" section to the system check
 - Add the ability to change the formatter for the logging params

## [0.7.2](https://pypi.org/project/hello-robot-stretch-body/0.7.2/) - January 26, 2024
 - Fix for wait_command() timeout not being respected (issue [#255](https://github.com/hello-robot/stretch_body/issues/255))
 - Improvements to the System Check tool, including:
    - Check if hello stepper self recognize correctly
    - Check if OV9782 camera seen for Stretch 3s
    - Update pip recommendations
    - Hide verbose printout
    - Add battery section to hardware check

## [0.7.0](https://pypi.org/project/hello-robot-stretch-body/0.7.0/) - January 25, 2024
 - Introduces P5 firmware protocol support
 - Revamped system check tool
 - Revamped wrist/tool management system
 - Started development on a self collision avoidance system
 - Utilities and CLI to introspect UVC cameras
 - Support for Dex Wrist 3

## [0.6.8](https://pypi.org/project/hello-robot-stretch-body/0.6.8/) - December 7, 2023
- Deprecate `robot.is_calibrated()` and introduce `robot.is_homed()` to match homing nomenclature used elsewhere ([details](https://github.com/hello-robot/stretch_body/pull/212))
- Added a `robot.wait_command()` method to easily block process execution until motion is completed ([details](https://github.com/hello-robot/stretch_body/pull/211))
- Added Dex Wrist URDF visualization to the `stretch_robot_urdf_visualizer.py` tool
- Automatic checking for updates in background while homing + user notification about out-of-date software in `stretch_robot_system_check.py` ([details](https://github.com/hello-robot/stretch_body/pull/213))
- Add mutex locking on Stretch Body + introduce `stretch_free_robot_process.py` tool ([details](https://github.com/hello-robot/stretch_body/pull/218))

## [0.6.2](https://github.com/hello-robot/stretch_body/pull/203) - September 11, 2023
- Introduces the new set of Gamepad modules to teleop stretch using the provided physical gamepad or any other UI input devices. This gamepad controller primarily uses velocity control.
- The stretch_gamepad_teleop.py tool is added and we are deprecating the old position control based stretch_xbox_teleop_controller.py
- The new controller now has a precision mode, allowing users to make fine-grain robot motions.

## [0.6.0](https://github.com/hello-robot/stretch_body/pull/206) - October 7, 2023
This is the initial productioon release that supports Prince batch.
- Introduces the P4 firmware protocol support
- Adds Prince batch params and new pimu IMU support  
## [0.5.0](https://github.com/hello-robot/stretch_body/pull/188) - July 11, 2023
- Introduces the **use_asyncio**  mode that will enable using asynchronous IO call methods to perform robot push/pull commands and RPC transactions with the help of [`asyncio`](https://docs.python.org/3/library/asyncio.html) to speed up the USB device communications. This mode can be toggled back to use the regular non-async IO calls by changing the stretch params `use_asyncio`.
- By default, the asyncio mode is enabled.
- Adds the P3 protocol support for all the Arduino devices (stepper, pimu, wacc).
- Also, for the asynchronous [transport layer](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/transport.py#L790) to work, the devices firmware will need to support V1 transport protocol that is supported with firmware only above [v0.5.0p3]() for all hello-* arduino devices. For older firmwares, the `async_io` would be disabled automatically
- The  DXLStatusThread of robot class is now separated into two threads: [DXLHeadStatusThread](https://github.com/hello-robot/stretch_body/blob/feature/transport_v1_asyncio/body/stretch_body/robot.py#L24) and [DXLEndofArmStatusThread](https://github.com/hello-robot/stretch_body/blob/feature/transport_v1_asyncio/body/stretch_body/robot.py#L51) (Both threads run at 15Hz).
- Now the robot monitor, trace, sentry, and collision manager handles stepping is moved out of the NonDXLStatusThread. Instead, it is moved to a new separate thread called [SystemMonitorThread](https://github.com/hello-robot/stretch_body/blob/feature/transport_v1_asyncio/body/stretch_body/robot.py#L140) is used to [step these handles](https://github.com/hello-robot/stretch_body/blob/feature/transport_v1_asyncio/body/stretch_body/robot.py#L140), which also runs at 25Hz.
- The Robot class can be [`startup()`](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py#L212) with the optional parameters to turn off some threads to save system resources.
- Added status_aux pull status RPCs feature for all the devices. [`motor_sync_cnt`](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py#L319) and [`motor_sync_queues`](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py#L317) status messages are populated with AUX pull status
- Now the RPC transactions queues is deprecated, instead [`do_pull_transaction_vX()`](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/transport.py#L272) and [`do_push_transaction_vX()`](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/transport.py#L189) methods are used from the [SyncTransactionHandler](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/transport.py#L75).
- Asynchournous RPC transactions are handled by [AsyncTransactionHandler](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/transport.py#L458) that create analougus async methods to the one present in [SyncTransactionHandler](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/transport.py#L75).



## [0.4.8](https://github.com/hello-robot/stretch_body/pull/148) - Sept 14, 2022

This is the initial production release that supports the Stretch RE2 (Mitski batch).

* This includes the [`robot_params_RE2V0.py`](https://github.com/hello-robot/stretch_body/blob/79be4b6441a1c451436aa65c23b1920282e3fed7/body/stretch_body/robot_params_RE2V0.py) which are the initial robot settings for the RE2 version of the product.

* It introduces the [PrismaticJoint](https://github.com/hello-robot/stretch_body/blob/79be4b6441a1c451436aa65c23b1920282e3fed7/body/stretch_body/prismatic_joint.py#L10) class which consolidates the common Arm and Lift functionality.

* It changes the units for guarded contact motion from approximate Newtons (suffix _N) to `effort_pct` - the pecentage [-100,100] of maximum current (A) that a joint should stop at. This change requires RE1 users to migrate their code and robot parameters. [See the forum post](https://forum.hello-robot.com/t/stretch-body-release-0-4-and-new-contact-model-units/476/1) for more details.

* It introduces [`mkdocs.yaml`](https://github.com/hello-robot/stretch_body/blob/79be4b6441a1c451436aa65c23b1920282e3fed7/mkdocs.yml) to support serving the repository documenation via MKDocs.

It introduces several new features and fixes several bugs, notably:

* [Adds `wait_until_at_setpoint()` to the Arm and Lift classes](https://github.com/hello-robot/stretch_body/commit/d15e3fb4416a5b296a184148df7b2045cf16027d)
* [Adds use of argparse with all tools](https://github.com/hello-robot/stretch_body/commit/c9c79d6fa08d0aec7d217e2e1d9a9d36b15145b1)
* [Moves Robot thread rates to YAML](https://github.com/hello-robot/stretch_body/commit/0fa82e852f98031064a4dbfba722af6da43bc992)
* [Cleans up the splined trajectory interface, enables velocity controlled splined trajectories for the Dynamixels](https://github.com/hello-robot/stretch_body/commit/13549e3662a3168c2a6f460f52c6577d0dbf5b5d)
* [Flags a warning for users incorrectly setting the homing offset on DXL servos](https://github.com/hello-robot/stretch_body/pull/151)



## [0.3.4](https://github.com/hello-robot/stretch_body/pull/142) - July 20, 2022

Release to add minor features and fix minor bugs:

* Add a `range_pad_t` parameter to allow for padding of hardstops for joint homing
* [Clean up tools and warnings to more consistent and legible #140](https://github.com/hello-robot/stretch_body/pull/140)


## [0.3.0](https://github.com/hello-robot/stretch_body/pull/129) - June 21, 2022
This release moves Stretch Body to use a new parameter management format. This change will require older systems to migrate their parameters to the new format. For systems that haven't yet migrated, Stretch Body will exit with a warning that they must migrate first by running `RE1_migrate_params.py`. See the [forum post](https://forum.hello-robot.com/t/425) for more details.

Features:

 - [New param management and RE1.5 support #135](https://github.com/hello-robot/stretch_body/pull/135)

## [0.2.1](https://github.com/hello-robot/stretch_body/pull/129) - January 6, 2022
Release to fix two bugs:

 - [Fix goto commands in the head jog tool #128](https://github.com/hello-robot/stretch_body/pull/128) - Fixes goto commands in the head jog tool
 - [Fix port_handler location #121](https://github.com/hello-robot/stretch_body/pull/121) - Fixes dxl buffer resetting under a serial communication failure

## [0.2.0](https://github.com/hello-robot/stretch_body/pull/118) - December 28, 2021
This release brings support for waypoint trajectories into master. Support for waypoint trajectories was built up over the last year in the [feature/waypoint_trajectories_py3](https://github.com/hello-robot/stretch_body/tree/feature/waypoint_trajectories_py3) branch, however, this branch couldn't be merged because the new functionality had flaky performance due to subtle bugs. This branch also attempted to introduce support for Python3 and timestamp synchronization. Support for Python3 and other features were merged in [v0.1.0](https://github.com/hello-robot/stretch_body/pull/35). The remaining features from this branch have been broken into 7 PRs, each targeting a specific device and squashing any previous bugs through functional and performance testing. They are:

 - [Introduce waypoint trajectory RPCs #98](https://github.com/hello-robot/stretch_body/pull/98)
 - [Add individual device threading #105](https://github.com/hello-robot/stretch_body/pull/105)
 - [Trajectory management classes #106](https://github.com/hello-robot/stretch_body/pull/106)
 - [Lift and arm trajectories #110](https://github.com/hello-robot/stretch_body/pull/110)
 - [Dynamixel trajectories #113](https://github.com/hello-robot/stretch_body/pull/113)
 - [Mobile base trajectories #114](https://github.com/hello-robot/stretch_body/pull/114)
 - [Whole body trajectories #115](https://github.com/hello-robot/stretch_body/pull/115)

This release also fixes several bugs. They are:

 - [fixed baud map bug #117](https://github.com/hello-robot/stretch_body/pull/117)
 - [fix contact thresh bug #116](https://github.com/hello-robot/stretch_body/pull/116)
 - [Fixed EndOfArm tools unittest #104](https://github.com/hello-robot/stretch_body/pull/104)

Testing:

Each PR in this release was tested on multiple robots, but was primarily tested on G2, on Python 2.7/Ubuntu 18.04.

## [0.1.11](https://github.com/hello-robot/stretch_body/pull/112) - October 4, 2021
This release gives Stretch Body the ability to support multiple firmware protocols, which at this moment is P0 and P1 firmware. P1 firmware builds on P0 to add waypoint trajectory support and a refactoring of controller functionality into classes. Additionally, this PR fixes how Dynamixel motors calculate velocity from encoder ticks.

Features:

 - [Support new firmware protocol (P1) #97](https://github.com/hello-robot/stretch_body/pull/97)

Bugfixes:

 - [Fix how negative dynamixel velocities are calculated #60](https://github.com/hello-robot/stretch_body/pull/60)

## [0.1.10](https://github.com/hello-robot/stretch_body/pull/102) - September 23, 2021
This release introduces 3 features:

1. [#68](https://github.com/hello-robot/stretch_body/pull/68) and [#95](https://github.com/hello-robot/stretch_body/pull/95): Introduces two new Jupyter notebooks that can be used to interactively explore working with Stretch. See [forum post](https://forum.hello-robot.com/t/jupyter-notebook-tutorials/298) for details.
1. [#94](https://github.com/hello-robot/stretch_body/pull/94): Introduces Github Action files and docs. Will be enabled in the future to automatically test PRs.
1. [#66](https://github.com/hello-robot/stretch_body/pull/66): Improves the statistics captured on Stretch Body's performance. Will be used to measure improvements to Stretch Body's communication with low level devices.

Bugfixes:

1. [#90](https://github.com/hello-robot/stretch_body/pull/90): Patches a bug where triggering `pimu.trigger_motor_sync()` at to high of a rate puts the robot into Runstop mode.
1. [#101](https://github.com/hello-robot/stretch_body/pull/101): Fixes bugs on startup of Dynamixel devices, ensures status is populated on startup of all devices, and add bool to `robot.startup()`

Testing:

All unit tests run on Python 2.7.17 on Ubuntu 18.04 on a Stretch RE1.

## [0.1.6](https://github.com/hello-robot/stretch_body/pull/85) - August 26, 2021
This release introduces these features:

* Revised soft limits and collision avoidance
* Added velocity interfaces for arm and lift

Bugfixes:

* Better error handling for DXL servos and their tools
* Fix bug where dxls maintain previous motion profile

## [0.1.4](https://github.com/hello-robot/stretch_body/pull/63) - July 20, 2021
This release introduces six features and several bugfixes. The features are:

- [Robot self-collision model and tutorial](https://github.com/hello-robot/stretch_body/pull/56)
- [Add ability to runstop individual DXL servos](https://github.com/hello-robot/stretch_body/pull/57)
- [Merged py2 and py3 tools](https://github.com/hello-robot/stretch_body/pull/59)
- [Added instructions for developing/testing Stretch Body](https://github.com/hello-robot/stretch_body/pull/61)
- [Collision avoidance tutorial](https://github.com/hello-robot/stretch_body/pull/62)
- [Improve realsense visualizer](https://github.com/hello-robot/stretch_body/pull/64)

Bugfixes:

- [Fix issue where user soft limits overwritten by collision models](https://github.com/hello-robot/stretch_body/pull/58)
- [Pin py2 deps to older version](https://github.com/hello-robot/stretch_body/pull/65)

## [0.1.0](https://github.com/hello-robot/stretch_body/pull/35) - May 30, 2021
This release introduces eight major features and several bugfixes. The features are:

1. [Python param management](https://github.com/hello-robot/stretch_body/pull/22)
2. [Configurable baud rate/GroupRead on Dynamixels](https://github.com/hello-robot/stretch_body/pull/20)
3. [Pluggable end effector tools](https://github.com/hello-robot/stretch_body/pull/32)
4. [Pluggable end effector support in Xbox Teleop](https://github.com/hello-robot/stretch_body/pull/30)
5. [Python logging](https://github.com/hello-robot/stretch_body/pull/32)
6. [Soft motion limit](https://github.com/hello-robot/stretch_body/pull/33)
7. [Self collision management](https://github.com/hello-robot/stretch_body/pull/34)
8. Unit testing framework - as part of each PR

Bugfixes:

1. [Multiturn enable_pos bug](https://github.com/hello-robot/stretch_body/pull/21) (#12) & [unit tests](https://github.com/hello-robot/stretch_body/pull/25)
2. [Misc bugs](https://github.com/hello-robot/stretch_body/pull/24)
3. Other - as part of the features

Testing:

All unit tests run on Python 2.7.17 on Ubuntu 18.04 on a Stretch RE1.
