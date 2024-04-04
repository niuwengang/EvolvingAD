#!/bin/bash





if [ $1 = "kitti" ]; then
    echo "数据集为kitti"
gnome-terminal --window --tab -e 'bash -c "source devel/setup.bash ;roslaunch urban_nav start.launch; exec bash"' \
--tab -e 'bash -c "sleep 5s;rosbag play   /home/g/workspace/dataset/2011_10_03_drive_0027_sync/kitti_2011_10_03_drive_0027_synced.bag; exec bash"'

elif [ $1 = "urbannav" ]; then
    echo "数据集为urbannav"
    gnome-terminal --window --tab -e 'bash -c "source devel/setup.bash ;roslaunch urban_nav start.launch; exec bash"' \
--tab -e 'bash -c "sleep 5s;rosbag play -r 0.5  /media/g/Elements/dataset/urban_nav/2019-04-28-20-58-02.bag; exec bash"'

else
    echo "设备运行"
fi
