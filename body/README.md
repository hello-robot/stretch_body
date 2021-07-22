Stretch Body
============

The stretch_body package provides a low level Python API to the Hello Robot Stretch RE1 hardware.

Installing
----------

To install stable Stretch Body for Python2, run:

```bash
$ python -m pip install --upgrade hello-robot-stretch-body
```

To install a pre-release of Stretch Body for Python2, run:

```bash
$ python -m pip install --upgrade --pre hello-robot-stretch-body
```

Please report feedback on the [Issue Tracker](https://github.com/hello-robot/stretch_body/issues) or the [Forum](https://forum.hello-robot.com/).

For Python3, substitute `python` with `python3`.

Running tests
-------------

There are a number of unit, functional, and performance tests within the `test/` folder, separated into test suites by different files. Suites are separated by a device or functionality within Stretch Body that is being tested.

In Python2, run `python -m unittest test.test_<suite-name>`. For Python3, substitute `python` with `python3`.

For example, to run the `stretch_body.robot.Robot` functional tests, run

```bash
$ git clone https://github.com/hello-robot/stretch_body.git
$ cd stretch_body/body
$ python -m unittest test.test_robot
```

Developing
----------

The source code for Stretch Body resides within the `stretch_body/` folder. You can install Stretch Body as "editable", and directly edit the source code to test changes.

In Python2, run `python -m pip install -e .` For Python3, substitute `python` with `python3`.

For example, to test changes to `stretch_body.robot.Robot`, run

```bash
$ git clone https://github.com/hello-robot/stretch_body.git
$ cd stretch_body/body
$ python -m pip install -e .
```

Now, make desired edits to the [stretch_body/body/stretch_body/robot.py](./stretch_body/robot.py) file. Software using Stretch Body is now using the modified `stretch_body.robot.Robot` class.

Deploying
---------

Increment the version number and run the `deploy.sh` script.
