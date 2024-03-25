# ad_common_project
复现常见的智驾算法或机器人算法

## 依赖
ros (含opencv4.0) 
```shell
#yaml
sudo apt-get install libyaml-cpp-dev
#jsk
sudo apt-get install ros-noetic-jsk-recognition-msgs
sudo apt-get install ros-noetic-jsk-rviz-plugins
```


## 编译
```
git clone git@github.com:niuwengang/ad_common_project.git
cd ad_common_project
git submodule init
git submodule update #获取子模块(部分不开源)
python3 scripts/build.py
```
 

## 规范
注释规范遵循Doxygen,git提交规范遵循Angular  
具体规则可见[必要规范](docs/必要规范.md)

## todo
1. 增加pangolin自定义可视化
2. 增加cpp-httplib应用
3. 图优化可视化

## 联系及反馈
niu_wengang@163.com或提交issue到仓库

测试



