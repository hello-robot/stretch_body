import setuptools
from os import listdir
from os.path import isfile, join

exec(open('stretch_body/version.py').read())

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="hello_robot_stretch_body",
    version=__version__,
    author="Hello Robot Inc.",
    author_email="support@hello-robot.com",
    description="Stretch RE1 low level  Python API",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_body",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2",
        "Operating System :: OS Independent",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)"
    ],
    install_requires=['numpy', 'scipy', 'matplotlib', 'ipython', 'jupyter', 'pandas', 'sympy', 'nose', 'PyYaml',
                      'inputs', 'drawnow', 'rplidar-roboticia', 'snakeviz', 'pyusb', 'SpeechRecognition', 'pixel-ring',
                      'click', 'cma', 'opencv-contrib-python', 'colorama', 'llvmlite==0.31.0', 'numba',
                      'scikit-image', 'open3d', 'pyrealsense2', 'pathlib', 'jsonschema==2.6.0', 'qtconsole==4.7.7',
                      'gitpython', 'urdfpy', 'psutil','hello-robot-stretch-body-tools','hello-robot-stretch-factory']
)
