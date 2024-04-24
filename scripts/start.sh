#!/bin/bash
gnome-terminal --window --tab -e 'bash -c "source devel/setup.bash ;roslaunch urban_nav start.launch; exec bash"' \
--tab -e 'bash -c "sleep 5s;rosbag play /media/g/Elements/dataset/kitti/kitti_2011_10_03_drive_0027_synced.bag; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash ;rosrun urban_nav preprocerss_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash ;rosrun urban_nav front_end_node; exec bash"' \
# --tab -e 'bash -c "source devel/setup.bash ;rosrun urban_nav back_end_node; exec bash"'


