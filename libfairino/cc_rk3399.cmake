set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(tools /usr/local/toolchain/rk3399/gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu)
set(CMAKE_C_COMPILER ${tools}/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${tools}/bin/aarch64-linux-gnu-g++)

#cmake -DCMAKE_TOOLCHAIN_FILE=cc_rk3399.cmake ..