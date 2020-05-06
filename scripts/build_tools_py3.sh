#! /bin/bash
cd ../tools_py3
rm -rf dist
rm -rf build
rm -rf *.egg-info
python3 setup.py sdist bdist_wheel
python -m twine upload dist/*

