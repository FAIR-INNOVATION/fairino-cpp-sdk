windows 下的编译方法：
1,win32
    cd build
    cmake .. -A win32
    # 编译release版本
    cmake --build . --config Release
    # 编译debug版本
    cmake --build .

1, x64
    cd build
    cmake .. -A x64
    # 编译release版本
    cmake --build . --config Release
    # 编译debug版本
    cmake --build .

linux 下的编译方法
    cd build
    cmake ..
    make