import setuptools
from stretch_body.version import __version__

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="hello_robot_stretch_body",
    version=__version__,
    author="Hello Robot Inc.",
    author_email="support@hello-robot.com",
    description="Stretch Body low level Python API",
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
                      'click', 'cma', 'colorama',
                      'scikit-image', 'open3d', 'pyrealsense2', 'pathlib', 'psutil', 'gitpython', 'urdfpy',
                      'opencv-contrib-python', 'renamed-opencv-python-inference-engine; python_version >= "3.0.0"', # resolve cv2 conflict for py3
                      'jsonschema==2.6.0; python_version < "3.0"', 'qtconsole==4.7.7; python_version < "3.0"', 'jupyter', # required by juypter
                      'llvmlite==0.31.0; python_version < "3.0"', 'numba', # numba required by stretch_funmap, depends on old llvmlite for py2
                      'terminado==0.8.3; python_version < "3.0"', # depend on old terminado for py2
                      'dynamixel-sdk>=3.1; python_version >= "3.0.0"', # py2 gets dynamixel-sdk through ROS
                      'pyyaml>=5.1', # required for yaml.FullLoader
                      'hello-robot-stretch-tool-share>=0.2.6', # defines other Stretch end effectors
                      'hello-robot-stretch-factory>=0.3.5','hello-robot-stretch-body-tools>=0.4.2'

                      ]
)
