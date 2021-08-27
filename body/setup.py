import setuptools
from stretch_body.version import __version__

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="hello_robot_stretch_body",
    version=__version__,
    author="Hello Robot Inc.",
    author_email="support@hello-robot.com",
    description="Stretch RE1 low level Python API",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_body",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2",
        "Operating System :: OS Independent",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)"
    ],
    install_requires=['numpy', 'scipy', 'matplotlib', 'ipython', 'pandas', 'sympy', 'nose',
                      'inputs', 'drawnow', 'rplidar-roboticia', 'snakeviz', 'pyusb', 'SpeechRecognition', 'pixel-ring',
                      'click', 'cma', 'opencv-contrib-python', 'colorama',
                      'scikit-image', 'open3d', 'pyrealsense2', 'pathlib', 'psutil', 'gitpython', 'urdfpy',
                      'jsonschema==2.6.0; python_version < "3.0"', 'qtconsole==4.7.7; python_version < "3.0"', 'jupyter', # required by juypter
                      'llvmlite==0.31.0; python_version < "3.0"', 'numba', # numba required by stretch_funmap, depends on old llvmlite for py2
                      'dynamixel-sdk>=3.1; python_version >= "3.0.0"', # py2 gets dynamixel-sdk through ROS
                      'pyyaml>=5.1', # required for yaml.FullLoader
                      'hello-robot-stretch-tool-share']
)
