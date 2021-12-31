#!/usr/bin/bash
CFD=$(realpath $(dirname $0))
cd $CFD
rm -rf Dependencies
mkdir -p Dependencies/PythonLibs
mkdir -p Dependencies/Libs
python_libs=(
    "$HOME/Rvbust/Install/RVBUST/RCI/Lib/Python/RVBUST/RCI/PyRCI.cpython-37m-x86_64-linux-gnu.so"
    "$HOME/Rvbust/Install/RVBUST/RPS/Lib/Python/RVBUST/RPS/PyRPS.cpython-37m-x86_64-linux-gnu.so"
    "$HOME/Rvbust/Install/RVBUST/Controllers/UR/Lib/Python/RVBUST/UR/PyUR.cpython-37m-x86_64-linux-gnu.so"
)
python_lib_dirs=(
    "$HOME/Rvbust/Install/RVBUST/RCI/Lib/Python/RVBUST"
    "$HOME/Rvbust/Install/RVBUST/RPS/Lib/Python/RVBUST"
    "$HOME/Rvbust/Install/RVBUST/Controllers/UR/Lib/Python/RVBUST"
)
for i in 0 1 2; do
    python_lib=${python_libs[$i]}
    python_lib_dir=${python_lib_dirs[$i]}
    echo "finding dependencies of $python_lib"
    cp -r $python_lib_dir Dependencies/PythonLibs
    bash ../../ShellScripts/CopyDependencies.sh $python_lib
done

# add change log
git log --pretty="%ad:%h %s" --date=short -n 100 >${CFD}/CHANGELOG
# python_scripts=$(App.py IViewer.py IVis.py)
python_scripts=$(ls *.py)
for python_script in ${python_scripts}; do
    if [ $(echo $python_script | grep Main.py) ]; then
        echo "skip Main.py"
        continue
    fi
    bash ../../ShellScripts/CythonModule.sh ${python_script}
done

rm -rf /tmp/RobotIVis
mkdir -p /tmp/RobotIVis
mv Dependencies /tmp/RobotIVis
mv *.so /tmp/RobotIVis

cp -r Resources /tmp/RobotIVis
cp Main.py /tmp/RobotIVis
cp *.sh /tmp/RobotIVis
cp CHANGELOG /tmp/RobotIVis
cp ReadMe.md /tmp/RobotIVis

cd /tmp
tar -czf "${CFD}/RobotIVis_$(date +%Y_%m_%d__%H_%M).tar.gz" RobotIVis
