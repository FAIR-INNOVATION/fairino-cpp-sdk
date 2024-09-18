#/bin/bash

rm cmake_install.cmake
rm CMakeCache.txt
rm Makefile
rm -rf CMakeFiles
rm -rf lib
rm -rf include



cmake -DCMAKE_TOOLCHAIN_FILE=cc_rk3568.cmake ..

make

cp -r bin lib
cp -r ../src/include/Robot include

