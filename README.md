# takktile_ros
### ROS drivers for the TakkTile tactile array

## Setup

1. Install TakkTile TakkFast USB drivers (see https://github.com/personalrobotics/TakkTile-usb)
    1. This will install TakkTile.py which is a dependency ~~(currently included as a symlink -- need to fix this)~~

1. Get code and compile

        cd ~/catkin_workspace/src
        catkin_init_workspace
        git clone https://github.com/personalrobotics/takktile_ros.git
        cd ~/catkin_workspace/
        catkin_make

1. set USB permissions

        sudo cp 71-takktile.rules /etc/udev/rules.d/

1. Run

        roscore &
        rosrun takktile_ros takktile_node.py &

## Plot
While takktile_node.py is running:

ROS Fuerte and earlier releases:
```
rxplot /takktile/calibrated/pressure[0]:pressure[1]:pressure[2]:pressure[3]:pressure[4]
```

ROS Fuerte and later releases:
```
rqt_plot /takktile/calibrated/pressure[0]:pressure[1]:pressure[2]:pressure[3]:pressure[4]
```

## Visualize on robot model
While takktile_node.py is running:
```
rosrun takktile_ros visualize_sensors.py /path/to/sensor_description.yml &
roslaunch urdf_tutorial display.launch gui:=True model:=/path/to/robot.urdf &
```
**NOTE:** requires numpy and pyyaml

### Yaml Layout File Format

```
[{ frame_id::str, sensors::[{ position::[float,float,float], normal::[float,float,float] }] }]
```

The order of the frame_id -> sensor mappings is important. The first `len([0].sensors)` pressure values are assigned to the first frame_id, and so on. So the order of the frame_ids must match the hardware order. The hardware order is determined by the UID of the USB board, the JP[1-5] wiring order, and the wiring order of the sensor pads themselves.
