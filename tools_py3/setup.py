import setuptools
from os import listdir
from os.path import isfile, join


with open("README.md", "r") as fh:
    long_description = fh.read()

script_path='./bin'
stretch_scripts={script_path+'/'+f for f in listdir(script_path) if isfile(join(script_path, f))}

setuptools.setup(
    name="hello_robot_stretch_body_tools_py3",
    version="0.0.4",
    author="Aaron Edsinger",
    author_email="aedsinger@hello-robot.com",
    description="Stretch Body Py3 Tools",
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
)

#classifiers = [
#    "Programming Language :: Python :: 2",
#    "License :: OSI Approved :: BSD License",
#    "Operating System :: OS Independent",
