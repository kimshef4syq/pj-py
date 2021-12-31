# 依赖
依赖OpenInventor的开源实现coin3d以及其python绑定库 `pivy`。
```shell
sudo apt install python3-pivy
python3 -m pip install -U pyside2
```
# 打包
运行 `Distribute.sh`, 在`App`目录生成`RobotIVis__xxx.tar.gz`

# 安装
将 `RobotIVis__xxx.tar.gz`复制到想要安装的机器上，解压
```
tar -xzvf RobotIVis*.tar.gz
```
运行里面的 `Install.sh`，桌面会出现快捷图标，双击即可运行程序。