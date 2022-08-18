=====
Usage
=====

Dataset Download
================

Download the dataset preview or the complete drive dataset (Gaimersheim, Ingolstadt or Munich) from the `A2D2 download page`_.
The dataset preview contains relatively short snippets, and a full drive dataset is quite large, for example for
Ingolstadt ~808GB:

- Bus Signals: 150MB
- Camera

    - Front Center: 79GB
    - Front Left: 73.5GB
    - Front Right: 73.6GB
    - Side Left: 79.0GB
    - Side Right: 75.7GB
    - Rear Center: 77.2GB

- Lidar

    - Front Center: 26.9GB
    - Front Left: 64.2GB
    - Front Right: 66.0GB
    - Side Left: 62.1GB
    - Side Right: 64.7GB
    - Rear Center: 66.5GB

If image data from some cameras are not needed in the rosbag to be created, there is no need to download them.
However, it is recommended to download all LiDAR datasets, e.g. the LiDAR dataset *front center*
contains the reflections of all sensors visible in the view *front center*.

The obtained directory structure should look like this:

.. code-block:: bash

    ./Ingolstadt
    ├── bus
    │   └── 20190401145936_bus_signals.json
    ├── camera
    │   ├── cam_front_center
    │   ├── cam_front_left
    │   ├── cam_front_right
    │   ├── cam_rear_center
    │   ├── cam_side_left
    │   └── cam_side_right
    ├── cams_lidars.json
    └── lidar
        ├── cam_front_center
        ├── cam_front_left
        ├── cam_front_right
        ├── cam_rear_center
        ├── cam_side_left
        └── cam_side_right

The sensor configuration file (cams_lidars.json) is also available at the `A2D2 download page`_.

.. _A2D2 download page: https://www.a2d2.audi/a2d2/en/download.html

Configuration
=============

The conversion can be parametrized via yaml files in the `a2d2_ros_preparer/config/`_ directory.
Here, the `default.yaml`_ contains all default parameter values.
The default values can then be overwritten in the `ingolstadt.yaml`_:

.. code-block:: yaml

    paths:
        source_sensor_data_directory: "/path/to/Ingolstadt"
        source_bus_signals_filepath: "/path/to/Ingolstadt/bus/20190401145936_bus_signals.json"
        source_sensor_configuration_filepath: "/path/to/Ingolstadt/cams_lidars.json"

        # rosbags and other artifacts are written in this directory
        target_directory: "/path/to/Ingolstadt_output"

    filter:
        # select only a certain time window for conversion (timestamps in seconds)
        start_timestamp: 1533906470
        stop_timestamp: 1554121580

Afterward, make sure that the yaml files are loaded in the `convert.launch`_ file:

.. code-block:: xml

    <launch>
        <group ns="converter/options">
            <rosparam file="$(find a2d2_ros_preparer)/config/default.yaml" />
            <rosparam file="$(find a2d2_ros_preparer)/config/ingolstadt.yaml" />
        </group>

        <node name="converter" pkg="a2d2_ros_preparer" type="converter" output="screen" required="true" args="" />
    </launch>

.. _a2d2_ros_preparer/config/: https://github.com/tum-gis/a2d2_ros_preparer/tree/main/a2d2_ros_preparer/config
.. _default.yaml: https://github.com/tum-gis/a2d2_ros_preparer/blob/main/a2d2_ros_preparer/config/default.yaml
.. _ingolstadt.yaml: https://github.com/tum-gis/a2d2_ros_preparer/blob/main/a2d2_ros_preparer/config/ingolstadt.yaml
.. _convert.launch: https://github.com/tum-gis/a2d2_ros_preparer/blob/main/a2d2_ros_preparer/launch/convert.launch

Converting
==========

If everything is configured correctly, simply run to generate the rosbag:

.. code-block:: bash

    source ./devel_isolated/setup.bash
    roslaunch a2d2_ros_preparer convert.launch

Depending on the amount of data, this might take some time and disk space.

Visualizing
===========

To inspect the rosbag with rviz, just run:

.. code-block:: bash

    roslaunch a2d2_ros_preparer visualize.launch bag_filename:=/path/to/Ingolstadt_output/driving_data.bag

|video|

.. |video| image:: https://j.gifs.com/PjMKkw.gif
    :alt: A2D2 ROS Preparer Demo
    :target: https://www.youtube.com/watch?v=uoTmNCU2IDM
