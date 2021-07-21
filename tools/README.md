Stretch Body Command Line Tools
===============================

This package provides Python tools that work with the Hello Robot Stretch Body package. These tools perform common tasks when working with Stretch RE1 (e.g. homing and stowing), and serve as tutorial code for working on various parts of the robot.

Installing
----------

To install stable Stretch Body Command Line Tools for Python2, run:

```bash
$ python -m pip install --upgrade hello-robot-stretch-body-tools
```

To install a pre-release of the Command Line Tools for Python2, run:

```bash
$ python -m pip install --upgrade --pre hello-robot-stretch-body-tools
```

Please report feedback on the [Issue Tracker](https://github.com/hello-robot/stretch_body/issues) or the [Forum](https://forum.hello-robot.com/).

For Python3, substitute `python` with `python3`.

Usage
-----

All of the command-line tools reside within the `bin/` folder. When this package is installed, they are accessible from anywhere as command-line tools. For example, to perform a robot system check, run:

```bash
$ stretch_robot_system_check.py
```

Developing
----------

The source code for the command-line tools resides within the `bin/` folder. You can install the tools package as "editable", and directly edit the source code to test changes.

In Python2, run `python -m pip install -e .` For Python3, substitute `python` with `python3`.

For example, to test changes to the  `stretch_robot_home.py` script, run

```bash
$ git clone https://github.com/hello-robot/stretch_body.git
$ cd stretch_body/tools
$ python -m pip install -e .
```

Now, make desired edits to the [stretch_robot_home.py](./bin/stretch_robot_home.py) file. Executing the script on the command-line will now run your modified version.

Deploying
---------

Increment the version number and run the `deploy.sh` script.
