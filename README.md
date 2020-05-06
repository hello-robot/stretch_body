# Stretch Body

The stretch_body repository includes Python packages that allow a developer to interact with the hardware of the Stretch RE1 robot. These packages are:
 * hello-robot-stretch_body
 * hello-robot-stretch_body-tools
 * hello-robot-stretch_body-tools_py3

These packages can be installed by:

```
pip install  hello-robot-stretch-body
pip install  hello-robot-stretch-body-tools
pip3 install hello-robot-stretch-body-tools-py3
```

User documentation can be found at https://hello-robot.github.io/stretch_body_guide/

## License
The license files found in the software packages here apply to the entire contents of their directories, which contains software exclusively for use with the Stretch RE1 mobile manipulator, which is a robot produced and sold by Hello Robot Inc.

Copyright 2020 Hello Robot Inc.

As stated by the detailed Licenses, "Patent and trademark rights are not licensed under this Public License."

For further information including inquiries about dual licensing, please contact Hello Robot Inc.
# Release Notes

## Package: hello-robot-stretch-body
Version 0.0.5 (2020/05/4)

* Added set_drive to Dynamixel interface
* Compatable with firmware 0.0.1.p0

Version 0.0.4 (05/04/2020)
* Initial public release for Guthrie robot
* Compatable with firmware 0.0.1.p0

## Package: hello-robot-stretch-body-tools
Version 0.0.5 (05/06/2020)
* Added stretch_audio_test.py
* Moved stretch_urdf_view.py to Py3 package
* Made stretch_hardware_echo.py pretty print to console

Version 0.0.4 (05/04/2020)
* Initial public release for Guthrie robot

## Package: hello-robot-stretch-body-tools
Version 0.0.3 (05/06/2020)
* Added stretch_urdf_view.py
* Initial public release for Guthrie robot
