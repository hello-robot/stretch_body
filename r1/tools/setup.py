import setuptools
from os.path import isfile
import glob

with open("README.md", "r") as fh:
    long_description = fh.read()

script_path='./bin'
ex_scripts = glob.glob(script_path+'/*.py') + glob.glob(script_path+'/*.sh')
stretch_scripts=[f for f in ex_scripts if isfile(f)]

setuptools.setup(
    name="hello-robot-stretch-body-tools",
    version="0.7.13",
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
    install_requires=['numpy>=1.24', 'scipy', 'matplotlib', 'ipython', 'pandas', 'sympy', 'nose', 'sh', 'packaging',
                      'inputs', 'drawnow', 'rplidar-roboticia', 'snakeviz', 'pyusb', 'SpeechRecognition', 'pixel-ring',
                      'click', 'cma', 'opencv-contrib-python', 'colorama', 'scikit-image', 'open3d', 'pyrealsense2', 'gitpython',
                      'xmltodict', 'filelock', 'pyaudio',
                      'pyglet == 1.4.10; python_version < "3.0"', 'trimesh==4.4.7', 'urchin', # urdfpy ==> urchin
                      'pyyaml>=5.1', # required for yaml.FullLoader
                     ]
)
