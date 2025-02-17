import setuptools
from stretch_body.version import __version__

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="hello-robot-stretch-body",
    version=__version__,
    author="Hello Robot Inc.",
    author_email="support@hello-robot.com",
    description="Stretch Body low level Python API",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_body",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)"
    ],
    install_requires=['numpy>=1.24', 'scipy', 'matplotlib', 'ipython', 'pandas', 'sympy', 'nose',
                      'inputs', 'drawnow', 'rplidar-roboticia', 'snakeviz', 'pyusb', 'SpeechRecognition', 'pixel-ring',
                      'click', 'cma', 'colorama', 'filelock',
                      'scikit-image', 'open3d', 'pyrealsense2', 'pathlib', 'psutil', 'gitpython', 'urchin', 'urdf_parser_py', # urdfpy ==> urchin
                      'opencv-contrib-python', 'renamed-opencv-python-inference-engine; python_version >= "3.0.0"', # resolve cv2 conflict for py3
                      'jupyter',
                      'numba', # numba required by stretch_funmap
                      'transforms3d>=0.4.2', # required by stretch_core
                      'dynamixel-sdk',
                      'pyyaml>=5.1', # required for yaml.FullLoader
                      'hello-robot-stretch-tool-share>=0.3.3', # defines other Stretch end effectors
                      'hello-robot-stretch-factory>=0.3.5','hello-robot-stretch-body-tools>=0.4.2',
                      'hello-robot-stretch-urdf>=0.0.19',
                      'aioserial', 'meshio','numpy-stl', 'pyrender', 'chime'
                      ]
)
