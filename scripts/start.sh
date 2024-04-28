# #!/bin/bash
gnome-terminal --window \
--tab -e 'bash -c "sleep 1s;roscore"' \
--tab -e 'bash -c "source devel/setup.bash ;rosrun evolving_ad ad_node; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash ;roslaunch evolving_ad start.launch; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5s;rosbag play /home/g/dataset/CA-20190828184706_blur_align.bag; exec bash"' \