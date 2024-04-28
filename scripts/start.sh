# #!/bin/bash
gnome-terminal --window \
--tab -e 'bash -c "sleep 1s;roscore"' \
--tab -e 'bash -c "source devel/setup.bash ;rosrun evolving_ad ad_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash ;roslaunch evolving_ad start.launch; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5s;rosbag  play  -r 0.5 /media/g/Elements/dataset/urban_loc/CA-20190828190411_blur_align.bag; exec bash"' \
#--tab -e 'bash -c "sleep 5s;rosbag play /media/g/Elements/dataset/urban_nav/2019-04-28-20-58-02.bag; exec bash"' \

#/media/g/Elements/dataset/urban_loc/CA-20190828190411_blur_align.bag