# #!/bin/bash

play_rate=1.0


if [ "$1" == "1" ]; then
    start_time=0
    duration_time=253
    dataset_path="/media/g/Elements/dataset/urban_loco/CA-20190828190411_blur_align.bag"
elif [ "$1" == "2.a" ]; then
    start_time=0
    duration_time=300
    dataset_path="/media/g/Elements/dataset/urban_loco/CA-20190828151211_blur_align-001.bag"
elif [ "$1" == "2.b" ]; then
    start_time=300
    duration_time=300
    dataset_path="/media/g/Elements/dataset/urban_loco/CA-20190828151211_blur_align-001.bag"
elif [ "$1" == "2.c" ]; then
    start_time=600
    duration_time=329
    dataset_path="/media/g/Elements/dataset/urban_loco/CA-20190828151211_blur_align-001.bag"
else
    echo "unknown param"
    exit 1 
fi

gnome-terminal --window \
--tab -e 'bash -c "sleep 1s;roscore; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash; rosrun evolving_ad ad_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash; roslaunch evolving_ad start.launch; exec bash"' \
--tab -e 'bash -c "sleep 5s; rosbag play -s '$start_time' -u '$duration_time' '"$dataset_path"'; exec bash"'
