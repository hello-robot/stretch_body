#! /bin/bash
#build and upload to pYPi
rm -rf dist
rm -rf build
rm -rf *.egg-info
python3 setup.py sdist bdist_wheel
python -m twine upload dist/*

#to install: pip3 install  hello-robot-stretch-body-tools-py3
#to uninstall: pip3 uninstall hello-robot-stretch-body-tools-py3
