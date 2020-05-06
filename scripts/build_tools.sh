#! /bin/bash
cd ../tools
rm -rf dist
rm -rf build
rm -rf *.egg-info
python setup.py sdist bdist_wheel
python -m twine upload dist/*

