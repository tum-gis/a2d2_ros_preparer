===============
Getting Started
===============

A2D2 ROS Preparer was developed and tested for `ROS Noetic`_ on Ubuntu 20.04 (focal).

.. _ROS Noetic: https://wiki.ros.org/noetic/Installation

Prerequisites
=============

With a ROS noetic installation, install the following packages:

.. code-block:: bash

    sudo apt install git python3-wstool python3-rosdep ninja-build ros-noetic-cnpy

Building
========

Create the workspace and clone the repo:

.. code-block:: bash

    mkdir catkin_ws
    cd catkin_ws

    source /opt/ros/noetic/setup.bash
    wstool init src
    git clone git@github.com:tum-gis/a2d2_ros_preparer.git src/a2d2_ros_preparer

Install the package dependencies and run catkin for compiling:

.. code-block:: bash

    sudo rosdep init
    rosdep update

    # if on Linux Mint 20, add the argument: --os=ubuntu:focal
    rosdep install --from-paths src --ignore-src -r --rosdistro=${ROS_DISTRO} -y

    catkin_make_isolated --install --use-ninja

In case you want to use `CLion`_ as IDE, run:

.. code-block:: bash

    source ./devel_isolated/setup.bash
    clion ./ &

.. _CLion: https://www.jetbrains.com/help/clion/ros-setup-tutorial.html
