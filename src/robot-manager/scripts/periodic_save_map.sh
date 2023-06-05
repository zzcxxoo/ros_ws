#! /bin/sh
while true
do

    current_time=$(date "+%Y.%m.%d-%H.%M.%S")
    echo "Current Time : $current_time"
    current_dir=/home/$USER/.robot/data/maps/current
    echo "Current Dir : $current_dir"
     
    #rosrun map_server map_saver -f latest-$current_time
    #need to add prefix directories here....
    #emulate map save
    #touch latest-$current_time.yaml
    #touch latest-$current_time.pgm
    #rosrun map_server map_saver -f $current_dir/$1 # latest-$current_time
    rosrun map_server map_saver -f map:=/projected_map $current_dir/current # latest-$current_time
    #emulate convert
    #touch latest-$current_time.jpg
    #convert $current_dir/$1.pgm $current_dir/$1.jpg
    #convert $current_dir/$1.pgm $current_dir/$1.png
    convert $current_dir/current.pgm $current_dir/current.png
    /home/$USER/.robot/config/mapfile_update  $current_dir/current.yaml 
    sleep 3

done

#need to add prefix directories here....
#tar cfz $current_dir/$1.tgz $current_dir/$1.jpg $current_dir/$1.pgm $current_dir/$1.png $current_dir/$1.yaml
#sed -i -- 's/\.pgm/\.png/g' $current_dir/$1.yaml

#remove old links
#need to add prefix directories here....
#ln -sfv $current_dir/latest-$current_time.jpg $current_dir/latest.jpg
#ln -sfv $current_dir/latest-$current_time.yaml $current_dir/latest.yaml
#ln -sfv $current_dir/latest-$current_time.pgm $current_dir/latest.pgm
#ln -sfv $current_dir/latest-$current_time.tgz $current_dir/latest.tgz

