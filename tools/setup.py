import setuptools
from os.path import isfile
import glob

with open("README.md", "r") as fh:
    long_description = fh.read()

script_path='./bin'
ex_scripts = glob.glob(script_path+'/*.py') + glob.glob(script_path+'/*.sh')
stretch_scripts=[f for f in ex_scripts if isfile(f)]

setuptools.setup(
    name="hello_robot_stretch_body_tools",
    version="0.4.6",
    author="Hello Robot Inc",
    author_email="support@hello-robot.com",
    description="Stretch Body Tools",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_body",
    scripts = stretch_scripts,
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "License :: OSI Approved :: Apache Software License"
    ],
    install_requires=['numpy', 'scipy', 'matplotlib', 'ipython', 'pandas', 'sympy', 'nose',
                      'inputs', 'drawnow', 'rplidar-roboticia', 'snakeviz', 'pyusb', 'SpeechRecognition', 'pixel-ring',
                      'click', 'cma', 'opencv-contrib-python', 'colorama', 'scikit-image', 'open3d', 'pyrealsense2',
                      'pyglet == 1.4.10; python_version < "3.0"', 'trimesh==3.6.38', 'urdfpy', # required for urdfpy viz in py2
                      'pyyaml>=5.1', # required for yaml.FullLoader
                      'hello-robot-stretch-body>=0.4.8']
)

#classifiers = [
#    "Programming Language :: Python :: 2",
#    "License :: OSI Approved :: BSD License",
#    "Operating System :: OS Independent",
