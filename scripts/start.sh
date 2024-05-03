# #!/bin/bash

play_rate=1.0
dataset_path="/media/g/Elements/dataset/urban_loc/CA-20190828190411_blur_align.bag"
#dataset_path="/media/g/Elements/dataset/urban_loc/CA-20190828151211_blur_align-001.bag"


gnome-terminal --window \
--tab -e 'bash -c "sleep 1s;roscore; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash; rosrun evolving_ad ad_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash; roslaunch evolving_ad start.launch; exec bash"' \
--tab -e 'bash -c "sleep 5s; rosbag play -r '$play_rate' '"$dataset_path"'; exec bash"'
