set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(tools /usr/local/toolchain/rk3568/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu)
set(CMAKE_C_COMPILER ${tools}/bin/aarch64-none-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${tools}/bin/aarch64-none-linux-gnu-g++)

#cmake -DCMAKE_TOOLCHAIN_FILE=cc_rk3568.cmake ..