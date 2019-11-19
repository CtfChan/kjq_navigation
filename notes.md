
# SSHing into robot
The SSID is ubiquityrobotXXXX where XXXX is part of the MAC address. The wifi password is robotseverywhere.

Once connected, it is possible to log into the Pi with ssh ubuntu@10.42.0.1 with a password of ubuntu. If you connect up a keyboard and mouse enter the password ubuntu at the prompt. 

If you are not running on one of our robots run sudo systemctl disable magni-base to ensure that our startup scripts get disabled. 

Set up lidar on pi
```
ssh ubuntu@10.42.0.1 #password: ubuntu
roslaunch ydlidar lidar.launch
```

Establish Connection:
Set VM network on bridged
use "hostname -I" to get your IP

On bot:
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=<ROBOT_IP>

on laptop:
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_HOSTNAME=<PC_IP>

on bot:
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

on laptop:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py


## Instructions for Demo 1
```
# Terminal 1: Launch stuff on pi
ssh ubuntu@10.42.0.1
roslaunch yd_lidar kjq.launch

# Terminal 2:Launch random drive on laptop
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.1 _turn:=1.0
```


## Instructions for Demo 2
```
# Terminal 1: Launch stuff on pi
ssh ubuntu@10.42.0.1
roslaunch yd_lidar kjq.launch

# Terminal 2: Launch random drive on laptop
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.1 _turn:=1.0

# Terminal 3: Close the gripper
rostopic pub /cmd_special std_msgs/String "close" -1
rostopic pub /cmd_special std_msgs/String "open" -1

```