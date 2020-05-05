import setuptools
from os import listdir
from os.path import isfile, join

exec(open('stretch_body/version.py').read())

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="hello_robot_stretch_body",
    version=__version__,
    author="Aaron Edsinger",
    author_email="aedsinger@hello-robot.com",
    description="Stretch RE1 low level API",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_body",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2",
        "Operating System :: OS Independent",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)"
    ],
)
