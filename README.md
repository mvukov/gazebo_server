[![CircleCI](https://circleci.com/gh/mvukov/gazebo_server.svg?style=svg)](https://circleci.com/gh/mvukov/gazebo_server)

This package provides a simple wrapper for Gazebo simulator. The package is
useful when you just want to run a simulator step-by-step or for a number of
steps and have full control when the steps are executed. In those situations
you are typically not interested in a robotics middleware (e.g. ROS),
multi-threading, potentially complex setup of the simulator, etc. The intention
here is to provide a concise and clear API to simplify setup and running of a
simulation. The package at the moment supports only one robot in the simulator.

`gazebo_server` provides functionality in C++ and Python 3 with a similar API.

Please take a look at tests to get the feeling how to get started.

The package has been tested with ROS Melodic and Ubuntu 18.04. In order to
compile the package you will need to install the following:

```bash
sudo apt install gazebo9 libeigen3-dev libgazebo9-dev libode-dev
sudo pip3 install pybind11
```
In addition, you will need to install at least CMake 3.12.
