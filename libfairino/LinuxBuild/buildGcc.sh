#/bin/bash

rm cmake_install.cmake
rm CMakeCache.txt
rm Makefile
rm -rf CMakeFiles
rm -rf lib
rm -rf include


#cmake ..

#cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-7 -DCMAKE_CXX_COMPILER=/usr/bin/g++-7 ..

#cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 ..

cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-11 -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 ..

make

cp -r bin lib
cp -r ../src/include/Robot include


