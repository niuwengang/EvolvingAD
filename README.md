# SimpleUrbanNOA

## 1.Introduce
A simple demo for urban navigation on autopilot. However, it's just a personal toy program. <br>





<div style="display: flex; justify-content: space-between;">
    <img src="/pic/display.jpeg" alt="图片alt" title="图片title" width="300px" height="200px" objectfit="cover" >
    <img src="/pic/display2.png" alt="图片alt" title="图片title" width="300px" height="200px" objectfit="cover" >
</div>

## 2.Installation
+ ros component
```
sudo apt-get install ros-noetic-imu-tools
sudo apt-get install ros-noetic-jsk-rviz-plugins
```
+ g2o <br>
```shell
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev 
cp src/urban_nav/setup_file/g2o.zip ${YOUR_SOFTWARE_FOLDER}
cd ${YOUR_SOFTWARE_FOLDER}
mkdir g2o
unzip g2o.zip -d g2o
mkdir build
cd build
cmake ..
make 
sudo make install 
```
+ yaml 0.70
```shell
git clone https://github.com/jbeder/yaml-cpp
cd yaml-cpp
git checkout yaml-cpp-0.7.0 #check to 0.70
mkdir build 
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make 
```
+ spdlog
```shell
sudo apt install libspdlog-dev
```
+ geographiclib
```
git clone https://github.com/geographiclib/geographiclib
cd  geographiclib
mkdir build
cd build
cmake ..
make 
sudo make  install
```
+ tensorRT
```

```


## 3.Pipeline


## 4.Acknowledgements
https://github.com/SmallMunich/nutonomy_pointpillars <br>
https://github.com/SMRT-AIST/fast_gicp <br>
https://github.com/Little-Potato-1990/localization_in_auto_driving <br>


## 5.Rules
code comments obey Doxygen rules <br>
git commits obey Angular rules <br>


## 6.Licence
The source code is released under GPLv3 license. <br>




## 7.Updates
**2024/04/25** <br>
tag v0.1.2 <br>
1. Added 3D object decetion (use pointpillars). <br>
2. Added dynamic object filter <br>
3. Added ground segement. <br>
4. Added 3d bouding box display. <br>


**2024/04/10** <br>
tag v0.1.1 <br>
1. Add backend for optimization (gnss's XYZ used as prior constraints, lidar odom used as interframe constraints). <br>
2. Added car model xiaomi su7. <br>

**2024/04/04** <br>
tag v0.1.0 <br>
1. Added a simple lidar odometry. <br>
2. Added visualization for odom and pointcloud. <br>



## 8.PreRelease
1. To add loop closure.<br>
2. To add traj save and compare to gt automatically. <br>
3. To add pointcloud saving and stitching. <br>
4. To add imu-preintegration. <br>
5. To add carla scence. <br>