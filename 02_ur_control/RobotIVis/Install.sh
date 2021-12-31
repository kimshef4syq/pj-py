# 安装系统依赖

sudo apt update
sudo apt install python3-pip python3-pivy
python3 -m pip install -U pyside2 scipy numpy

CFD=$(realpath $(dirname $0))

cat > "${CFD}/RobotIVis.desktop" << EOF
[Desktop Entry]
Name=RobotIVis
Type=Application
Exec=bash ${CFD}/StartRobotIVis.sh
Icon=${CFD}/Resources/Icons/Robot.svg
Terminal=false
Categories=Development
StartupNotify=true
Comment="An esay to use robot simulation and programing software"
Version=$(date "+%Y/%m/%d %H:%M:%S")
EOF

cp ${CFD}/RobotIVis.desktop $HOME/Desktop

PYTHONPATH=
LD_LIBRARY_PATH=

cat > "${CFD}/StartRobotIVis.sh" << EOF
CFD=$(realpath $(dirname $0))
cd $CFD
echo "current working directory is \${CFD}"
LD_LIBRARY_PATH=
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${CFD}/Dependencies/Libs
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib

PYTHONPATH=
export PYTHONPATH=\$PYTHONPATH:/opt/RVBUST/Vis/Python
export PYTHONPATH=\$PYTHONPATH:${CFD}/Dependencies/PythonLibs

python3 Main.py

echo "press any key to exit"
read _
EOF