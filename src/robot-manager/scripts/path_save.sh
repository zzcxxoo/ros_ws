#! /bin/sh
current_time=$(date "+%Y.%m.%d-%H.%M.%S")
echo "Current Time : $current_time"
current_dir=/home/$USER/.robot/data/maps/$1
echo "Current Dir : $current_dir"
# cd $current_dir
mkdir -p $current_dir/path
cp $current_dir/current_path_record.yaml $current_dir/path/$2.yaml
cp $current_dir/current_path_record.yaml $current_dir/path/$2.csv
# cp $current_dir/current_path_record_smooth.yaml $current_dir/path/$2_smooth.yaml
rosparam set /path_saved true


