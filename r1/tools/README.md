Stretch Body Command Line Tools
===============================

This package provides Python tools that work with the Hello Robot Stretch Body package. These tools perform common tasks when working with Stretch (e.g. homing and stowing), and serve as tutorial code for working on various parts of the robot. This package comes pre-installed on Stretch robots. A tutorial for using this package can be found [on the docs](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_command_line_tools/).

Installing
----------

This package comes pre-installed on Stretch robots. To install or upgrade to a stable Stretch Body Command Line Tools for Python3, run:

```bash
$ python3 -m pip install --upgrade hello-robot-stretch-body-tools
```

To install or upgrade to a pre-release of the Command Line Tools for Python3, run:

```bash
$ python3 -m pip install --upgrade --pre hello-robot-stretch-body-tools
```

Please report feedback on the [Issue Tracker](https://github.com/hello-robot/stretch_body/issues) or the [Forum](https://forum.hello-robot.com/).

Usage
-----

All of the command-line tools reside within the `bin/` folder. When this package is installed, they are accessible from anywhere as command-line tools. For example, to perform a robot system check, run:

```bash
$ stretch_robot_system_check.py
```

For more info on these tools, see the [tutorial](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_command_line_tools/).

Developing
----------

The source code for the command-line tools resides within the `bin/` folder. You can install the tools package as "editable", and directly edit the source code to test changes.

In Python3, run `python3 -m pip install -e .`

For example, to test changes to the  `stretch_robot_home.py` script, run

```bash
$ python3 -m pip uninstall hello-robot-stretch-body-tools # ensure previous Stretch Body Tools installations are removed
$ git clone https://github.com/hello-robot/stretch_body.git
$ cd stretch_body/tools
$ python3 -m pip install -e .
```

Now, make desired edits to the [stretch_robot_home.py](./bin/stretch_robot_home.py) file. Executing the script on the command-line will now run your modified version.

Deploying
---------

Increment the version number and run the `deploy.sh` script. Verify the new release is reflected [on PyPI](https://pypi.org/project/hello-robot-stretch-body-tools/).
