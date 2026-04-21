#!/bin/bash
set -e

./setupSymlink.sh

# Install the python packages for venv
virtualenv --python=python2.7 ./temporal-planning-main

# Activate venv
cd temporal-planning-main
source bin/activate
pip install -r requirements.txt

cd temporal-planning
./build.sh
python2.7 fd_copy/build.py release64

