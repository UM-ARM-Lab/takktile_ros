takktile-ros
============

ros drivers for the TakkTile tactile array

1.) Install TakkTile TakkFast USB drivers (see https://github.com/TakkTile/TakkTile-usb)

Follow instructions at [http://www.takktile.com/tutorial:takkfast]

This will install TakkTile.py which is a dependency (currently included as a symlink -- need to fix this)



2) Get code

> git clone https://github.com/UM-ARM-Lab/takktile_ros.git

Change the symlink:

    cd ~/catkin_ws/src/takktile_ros/src
    ln -sf [PATH TO TakkTile.py] TakkTile.py

3) Compile

`catkin_make (or catkin build)`

4) set USB permissions

`sudo cp 71-takktile.rules /etc/udev/rules.d/`

5) Run

`rosrun takktile_ros takktile_node.py`

6) Plot (in another terminal while takktile_node.py is running)

`rqt_plot /takktile/calibrated/pressure[0]:pressure[1]:pressure[2]:pressure[3]:pressure[4]`
