## Basic info

From original repo:
- Dashgo D1 14.04
- Version:v2.0
- Date:2017-11-23

## TIERS bot

We are controlling the dashgo platform using Ubuntu 18.04\ ROS Melodic.

### Install dependencies:
```
sudo apt install ros-melodic-serial ros-melodic-yocs-velocity-smoother 
```

### Run dashbo driver

First make the node executable:
```
chmod +x $(find dashgo_d1/dashgo_driver/nodes/dashgo_driver.py)
```

Edit the port in the config file (first line)
```
nano $(find dashgo_d1/dashgo_driver/config/my_dashgo_params.yaml)
```

Make sure user is in `dialout` group:
```
sudo usermod -aG dialout $USER
```

And run the demo
```
roslaunch dashgo_driver demo.launch
```

In order to control the platform with keyboard teleop, install it with
```
sudo apt install ros-melodic-teleop-twist-keyboard
```
and run
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
