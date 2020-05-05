import setuptools
from os import listdir
from os.path import isfile, join


with open("README.md", "r") as fh:
    long_description = fh.read()

script_path='./stretch_body_tools'
stretch_scripts={script_path+'/'+f for f in listdir(script_path) if isfile(join(script_path, f))}

setuptools.setup(
    name="hello_robot_stretch_body_tools",
    version="0.0.4",
    author="Aaron Edsinger",
    author_email="aedsinger@hello-robot.com",
    description="Stretch Body Tools",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_body",
    scripts = stretch_scripts,
    packages=setuptools.find_packages(),
    license="Apache License 2.0",
    classifiers=[
        "Programming Language :: Python :: 2",
        "Operating System :: OS Independent",
    ],
)

#classifiers = [
#    "Programming Language :: Python :: 2",
#    "License :: OSI Approved :: BSD License",
#    "Operating System :: OS Independent",
