#!/bin/bash

set -e

pip -q install 'pylint<2.0.0'
. /root/catkin_ws/install/setup.bash
find * -iname '*.py' | xargs python -m pylint
