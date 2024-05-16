# #!/bin/bash

play_rate=1.0


if [ "$1" == "1" ]; then
    dataset_path="/media/g/Elements/dataset/urban_loco/CABayBridge.bag"
elif [ "$1" == "2" ]; then
    dataset_path="/media/g/Elements/dataset/urban_loco/CAGoldenBridge.bag"
elif [ "$1" == "2" ]; then
    dataset_path="/media/g/Elements/dataset/urban_loco/CALombardStreet.bag"
else
    echo "unknown param"
    exit 1 
fi

gnome-terminal --window \
--tab -e 'bash -c "sleep 1s;roscore; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash; rosrun evolving_ad ad_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash; roslaunch evolving_ad start.launch; exec bash"' \
--tab -e 'bash -c "sleep 5s; rosbag play -s '$start_time' -u '$duration_time' '"$dataset_path"'; exec bash"'
