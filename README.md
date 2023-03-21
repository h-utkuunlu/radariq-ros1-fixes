# How to get started with Radar IQ and ROS.

Note: These instructions have been tested with ROS kinetic but should work with other version of ROS.

Some prior knowledge about how ROS works is assumed in this tutorial. Newcomers to ROS should check out the getting started guide on the [offical ROS wiki](http://wiki.ros.org).

![RViz Screenshot](docs/rviz.png "RViz Screenshot")

In this tutorial you will learn how to:
1. Configure Linux so that the RadarIQ module is presented consistently as a device.
2. Setup a new ROS workspace for the RadarIQ module.
3. Visualize the data using RViz.
4. Configure settings and incorporate the module into an existing ROS project.

# Install Python SDK
Install the Python SDK by running

``pip install radariq``

# Configure Linux
By default USB devices are presented to the Linux Kernel as /dev/ttyACMxx (where xx is a number).
The exact device name changes depending on the order in which the USB devices are connected or detected by the kernel.

In order to make USB devices easier to use, it is advisable to ensure they are always mapped to a common system name.

RadarIQ comes with a script for making the RadarIQ module present itself as /dev/radariq.

## Running the script
1. Change to the radariq_ros package.

   ``cd radariq_ros/scripts``

2. Make the setup scripts executable.

   ``chmod +x setup.sh``

3. Run the setup script and choose the "Install" option

   ```./setup.sh```

4. Plug the RadarIQ module into the ROS machine and check that it was correctly detected.

   ``ls /dev | grep "radariq"``

If the RadarIQ module was correctly detected ``radariq`` should be shown.


# Setup a new ROS workspace for RadarIQ
Note: These instructions assume your workspace is named ``catkin_ws``. If your workspace is named differently, adjust accordingly.

1. Create a new catkin workspace as per the [getting started instructions on the ROS website](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

2.  Make sure you are in the catkin_ws directory and have sourced the setup script for the workspace.

   ``cd ~/catkin_ws``
   
   ``source devel/setup.bash``
   
3. Clone the RadarIQ ROS repository into the src directory.

   ``git clone https://github.com/radariq/ros1 ./src/radariq_ros``

4. Run catkin_make to make the workspace.

   ``catkin_make``

# Visualize the data using RViz
RadarIQ pointcloud data can be started and visualised using the ROS RViz tool.

1. From your ROS workspace run

   ``roslaunch radariq_ros view_radariq_pointcloud.launch``

The RadarIQ module should start up and RViz should open, showing a point cloud of detections.

2. From your ROS workspace run

   ``roslaunch radariq_ros view_radariq_objects.launch``

The RadarIQ module should start up and RViz should open, showing a object representations of the detections.

# Incorporate into an existing ROS project
These instructions explain how to incorporate RadarIQ into an existing ROS workspace using a provided sample application
 which simply echos the data out to the ROS log.

## Steps

1. Adjust the module settings

   The module settings such as distance filters, angle filters etc can adjusted using a launch file.

   The default launch file is located: ``src/radariq_ros/launch/radariq.launch``

2. Start ROS core

   ``roscore``

3. Start the publisher node

   ``rosrun radariq_ros publisher_node.py``

4. Start the test application

   ``rosrun radariq_ros test_radariq.launch``

Raw data should be echoed out to the ROS log (see ``scripts/example_application.py`)
