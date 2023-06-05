#! /bin/sh
ping 192.168.3.200 -c 3
if [  $? = 0 ] 
  then
    roslaunch robot_manager robot_tf.launch use_hesai:=false;
  else 
    roslaunch robot_manager robot_tf.launch use_hesai:=true;
fi