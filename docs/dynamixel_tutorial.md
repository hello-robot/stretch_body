![](./images/banner.png)
# Working with Dynamixel Servos

In this tutorial we will cover the basics required to work with Dynamixel servos and Stretch.

**Note**: This tutorial applies to Stretch Body v0.1.x or greater

## Overview

Stretch comes with two Dynamixel buses - one for the head and one for the end-of-arm:

```bash
ls  /dev/hello-dynamixel-*
/dev/hello-dynamixel-head  /dev/hello-dynamixel-wrist
```

Typically users will interact with these devices through either the [Head](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/head.py) or [EndOfArm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py) interfaces. The EndOfArm interface may be extended to support custom tools, as described in the Stretch Body Tool Change Tutorial.

In some cases users will need to work directly with the servos from the command line. The sections below covers these tools 

## Servo Tools

### Jogging the Servos

You can directly command each servo using the command line tool `RE1_dynamixel_servo_jog.py`. This can be useful for debugging new servos added to the end-of-arm tool during system bring-up. For example, to command the head pan servo:

```bash
RE1_dynamixel_jog.py /dev/hello-dynamixel-head 11
[Dynamixel ID:011] ping Succeeded. Dynamixel model number : 1080
------ MENU -------
m: menu
a: increment position 50 tick
b: decrement position 50 tick
A: increment position 500 ticks
B: decrement position 500 ticks
v: set profile velocity
u: set profile acceleration
z: zero position
h: show homing offset
o: zero homing offset
q: got to position
p: ping
r: reboot
w: set max pwm
t: set max temp
i: set id
d: disable torque
e: enable torque
-------------------

```

### Rebooting the Servos

Under high-load conditions the servos may enter an error state to protect themselves from thermal overload. In this case, the red LED on the servo will flash (if visible). In addition, the servo will be unresponsive to motion commands. In this case, allow the overheating servo to cool down and reboot the servos using the `stretch_robot_dynamixel_reboot.py` tool: 

```bash
stretch_robot_dynamixel_reboot.py 
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.

---- Rebooting Head ---- 
[Dynamixel ID:011] Reboot Succeeded.
[Dynamixel ID:012] Reboot Succeeded.
---- Rebooting Wrist ---- 
[Dynamixel ID:013] Reboot Succeeded.
[Dynamixel ID:014] Reboot Succeeded.
```

### Identify Servos on the Bus

If it is unclear which servos are on the bus, and at what baud rate, you can use the `RE1_dynamixel_id_scan.py` tool. Here we see that the two head servos are at ID 11 and 12 at baud 57600.

```bash
RE1_dynamixel_id_scan.py /dev/hello-dynamixel-head --baud 57600
Scanning bus /dev/hello-dynamixel-head at baud rate 57600
----------------------------------------------------------
[Dynamixel ID:000] ping Failed.
[Dynamixel ID:001] ping Failed.
[Dynamixel ID:002] ping Failed.
[Dynamixel ID:003] ping Failed.
[Dynamixel ID:004] ping Failed.
[Dynamixel ID:005] ping Failed.
[Dynamixel ID:006] ping Failed.
[Dynamixel ID:007] ping Failed.
[Dynamixel ID:008] ping Failed.
[Dynamixel ID:009] ping Failed.
[Dynamixel ID:010] ping Failed.
[Dynamixel ID:011] ping Succeeded. Dynamixel model number : 1080
[Dynamixel ID:012] ping Succeeded. Dynamixel model number : 1060
[Dynamixel ID:013] ping Failed.
[Dynamixel ID:014] ping Failed.
[Dynamixel ID:015] ping Failed.
[Dynamixel ID:016] ping Failed.
[Dynamixel ID:017] ping Failed.
[Dynamixel ID:018] ping Failed.
[Dynamixel ID:019] ping Failed.
[Dynamixel ID:020] ping Failed.
[Dynamixel ID:021] ping Failed.
[Dynamixel ID:022] ping Failed.
[Dynamixel ID:023] ping Failed.
[Dynamixel ID:024] ping Failed.

```

### Setting the Servo Baud Rate

Dynamixel servos come with baudrate=57600 from the factory.  When adding your own servos to the end-of-arm tool, you may want to set the servo ID using the `RE1_dynamixel_set_baud.py` tool. For example:

```bash
RE1_dynamixel_set_baud.py /dev/hello-dynamixel-wrist 13 115200
---------------------
Checking servo current baud for 57600
----
Identified current baud of 57600. Changing baud to 115200
Success at changing baud
```

### Setting the Servo ID

Dynamixel servos come with ID=1 from the factory. When adding your own servos to the end-of-arm tool, you may want to set the servo ID using the `RE1_dynamixel_id_change.py` tool. For example:

```bash
RE1_dynamixel_id_change.py /dev/hello-dynamixel-wrist 1 13 --baud 115200
[Dynamixel ID:001] ping Succeeded. Dynamixel model number : 1080
Ready to change ID 1 to 13. Hit enter to continue:

[Dynamixel ID:013] ping Succeeded. Dynamixel model number : 1080
Success at setting ID to 13

```



## 

------
.<div align="center"> All materials are Copyright 2020 by Hello Robot Inc. The Stretch RE1 robot has patents pending</div>