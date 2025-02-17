#! /bin/bash
#build and upload to pYPi
rm -rf dist
rm -rf build
rm -rf *.egg-info
python3 setup.py sdist bdist_wheel
python3 -m twine upload dist/*
#to install: pip2 install  hello-robot-stretch-body-tools
#to uninstall: pip2 uninstall hello-robot-stretch-body-tools
