# usage: ./CythonModule Path/To/Your/Python/Script.py
# $1 : files
echo $1
stem=$(dirname $1)/$(basename -s .py $1)
cythonize -3 --directive embedsignature=True $1
echo "generating ${stem}$(python3-config --extension-suffix) ..."
gcc -O3 -Wall -shared -fPIC $(python3-config --includes)  \
    ${stem}.c -o ${stem}$(python3-config --extension-suffix)
rm ${stem}.c