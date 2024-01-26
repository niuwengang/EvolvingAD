# common_project
通用工程框架:积累复现常见算法及个人开源闭源算法的通用库

## 依赖
ros (含opencv4.0) 
```shell
#yaml
sudo apt-get install libyaml-cpp-dev
#json
sudo apt-get install libjsoncpp-dev

```


## 编译
```
git clone git@github.com:niuwengang/ad_common_project.git
cd ad_common_project
git submodule init
git submodule update
python3 scripts/build.py
python3 scripts/run.py ${exec_bin} # exec_bin替换

```
 

## 规格  
注释规范Doxygen 
提交消息规范Angular 

## todo
增加pangolin

