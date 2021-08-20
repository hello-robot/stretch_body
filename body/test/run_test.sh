#! /bin/bash
cd ..
cp -rf stretch_body/* $HOME/.local/lib/python2.7/site-packages/stretch_body
echo "Copied python to $HOME/.local/lib/python2.7/site-packages/stretch_body/"
cd test
python -m unittest test_collisions
