# SimpleUrbanNOA

## 1.Introduce
A simple demo for urban navigate on autopilot. <br>

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


## 3.Pipeline


## 4.Acknowledgements



## 5.Licence
The source code is released under GPLv3 license. <br>

## 6.Updates

**2024/04/10** <br>
tag v0.1.1 <br>
1. Add backend for optimizer (gnss's XYZ used as prior constraints, lidar odom used as interframe constraints). <br>
2. Display car modle xiaomi su7. <br>

**2024/04/04** <br>
tag v0.1.0 <br>
1. Added a simple lidar odometry. <br>
2. Display odom and cloud. <br>
<br>