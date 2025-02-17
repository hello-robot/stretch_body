Stretch Body
============

The stretch_body package provides a low level Python API to the Hello Robot Stretch hardware. This package comes pre-installed on Stretch robots. Tutorials for using this package can be found [on the docs](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/).

Installing
----------

This package comes pre-installed on Stretch robots. To install or upgrade to a stable Stretch Body for Python3, run:

```bash
$ python3 -m pip install --upgrade hello-robot-stretch-body
```

To install or upgrade to a pre-release of Stretch Body for Python3, run:

```bash
$ python3 -m pip install --upgrade --pre hello-robot-stretch-body
```

Please report feedback on the [Issue Tracker](https://github.com/hello-robot/stretch_body/issues) or the [Forum](https://forum.hello-robot.com/).

Running tests
-------------

There are a number of unit, functional, and performance tests within the `test/` folder, separated into test suites by different files. Suites are separated by a device or functionality within Stretch Body that is being tested.

In Python3, run `python3 -m unittest test.test_<suite-name>`.

For example, to run the `stretch_body.robot.Robot` functional tests, run:

```bash
$ git clone https://github.com/hello-robot/stretch_body.git
$ cd stretch_body/body
$ python3 -m unittest test.test_robot
```

Developing
----------

The source code for Stretch Body resides within the `stretch_body/` folder. You can install Stretch Body as "editable", and directly edit the source code to test changes.

In Python3, run `python3 -m pip install -e .`

For example, to test changes to `stretch_body.robot.Robot`, run:

```bash
$ python3 -m pip uninstall hello-robot-stretch-body # ensure previous Stretch Body installations are removed
$ git clone https://github.com/hello-robot/stretch_body.git
$ cd stretch_body/body
$ python3 -m pip install -e .
```

Now, make desired edits to the [stretch_body/body/stretch_body/robot.py](./stretch_body/robot.py) file. Software using Stretch Body is now using the modified `stretch_body.robot.Robot` class.

Deploying
---------

Increment the version number and run the `deploy.sh` script. Verify the new release is reflected [on PyPI](https://pypi.org/project/hello-robot-stretch-body/).
