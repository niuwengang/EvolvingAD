# #!/bin/bash
# gnome-terminal --window --tab -e 'bash -c "source devel/setup.bash ;roslaunch evolving_ad start.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5s;rosbag play /media/g/Elements/dataset/kitti/kitti_2011_10_03_drive_0027_synced.bag; exec bash"' \
# --tab -e 'bash -c "source devel/setup.bash ;rosrun evolving_ad preprocerss_node; exec bash"' \
# --tab -e 'bash -c "source devel/setup.bash ;rosrun evolving_ad front_end_node; exec bash"' \
# --tab -e 'bash -c "source devel/setup.bash ;rosrun evolving_ad back_end_node; exec bash"'

gnome-terminal --window \
--tab -e 'bash -c "sleep 1s;roscore"' \
--tab -e 'bash -c "source devel/setup.bash ;rosrun evolving_ad ad_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash ;roslaunch evolving_ad start.launch; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5s;rosbag play /home/g/dataset/CA-20190828184706_blur_align.bag; exec bash"' \