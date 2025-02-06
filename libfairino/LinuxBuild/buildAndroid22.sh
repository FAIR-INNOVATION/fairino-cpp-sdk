#/bin/bash

rm cmake_install.cmake
rm CMakeCache.txt
rm Makefile
rm -rf CMakeFiles
rm -rf lib
rm -rf include


export ANDROID_NDK=/usr/android-ndk-r22b


cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
	-DANDROID_ABI="armeabi-v7a" \
	-DANDROID_NDK=$ANDROID_NDK \
	-DANDROID_PLATFORM=android-24 \
	..

make


cp -r bin lib
cp -r ../src/include/Robot-CN include