

ssh ubuntu@10.42.0.1
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=10.42.0.1
killall roscore 
roscore &

on computer:
export ROS_MASTER_URI=http://10.42.01:11311
export ROS_HOSTNAME=`hostname -I`

