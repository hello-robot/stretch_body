#! /bin/bash
#build and upload to pYPi
rm -rf dist
rm -rf build
rm -rf *.egg-info
python setup.py sdist bdist_wheel
python -m twine upload dist/*
#to install: pip install  hello-robot-stretch-body
#to uninstall: pip uninstall hello-robot-stretch-body
