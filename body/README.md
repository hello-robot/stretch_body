Stretch Body
============

The stretch_body package provides a low level Python API to the Hello Robot Stretch RE1 hardware.

Running tests
-------------

There are a number of unit, functional, and performance tests within the `test/` folder, separated into test suites by different files. Suites are separated by a device or functionality within Stretch Body that is being tested.

In Python2, run

```
$ python -m unittest test.test_<suite-name>
```

For example, to run the `stretch_body.robot.Robot` functional tests, run

```
$ python -m unittest test.test_robot
```
