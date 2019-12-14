#!/bin/bash
GAZEBO_SETUP_SCRIPT=$(pkg-config --variable=prefix gazebo)/share/gazebo/setup.sh
. "${GAZEBO_SETUP_SCRIPT}"
