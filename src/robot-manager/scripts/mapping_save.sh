#! /bin/sh

echo "params: $1"

convert $1.pgm $1.jpg
convert $1.pgm $1.png
/home/$USER/.robot/config/mapfile_update  $1.yaml
cp $1.png $1.png.orig
tar cfz $1.tgz $1.jpg $1.pgm $1.png $1.yaml
sed -i -- 's/\.pgm/\_filtered.png/g' $1.yaml

rosparam set /path_saved true