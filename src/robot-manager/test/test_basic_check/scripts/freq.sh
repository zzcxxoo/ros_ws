#! /usr/bin/bash
source /home/hgy/9tian_ws/devel/setup.bash
rostopic hz /$1 > /tmp/$1 &
cmd=$!
sleep 4
kill -15 $cmd
sleep 1