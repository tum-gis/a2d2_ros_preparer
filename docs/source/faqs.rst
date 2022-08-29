==========================
Frequently Asked Questions
==========================

Why not simply use the *A2D2 to ROS* package?
=============================================

The `A2D2 to ROS`_ package already exists and is good. However, we wanted to run `Cartographer`_'s SLAM on the A2D2
dataset and thus needed to do some :doc:`preparations <../dataset_preparation>`.

.. _A2D2 to ROS: https://gitlab.com/MaplessAI/external/a2d2_to_ros/
.. _Cartographer: https://github.com/cartographer-project/cartographer

What about a ROS2 bag?
======================

The obtained ROS1 bag can be converted to ROS2 using the `rosbags tool`_.

.. _rosbags tool: https://gitlab.com/ternaris/rosbags
