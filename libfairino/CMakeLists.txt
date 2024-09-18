cmake_minimum_required(VERSION 3.10.0)

project(fairino_cppSDK_linux VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置编译开关
if(WIN32) 
    message("this windows platform")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    add_definitions(-DWINDOWS_OPTION)
elseif(UNIX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    message("this is unix-like system")
    add_definitions(-DLINUX_OPTION)
endif()

set(BASE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/src/include/Base)
set(LOG_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/src/include/Log)
set(ROBOT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/src/include/Robot)
#set(ROBOT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/EnInclude)
set(TCP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/src/include/TcpClient)
set(XMLRPC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/src/include/XmlRpc)

set(BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/src/Base)
set(LOG_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/src/Log)
set(ROBOT_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/src/Robot)
set(TCP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/src/TcpClient)
set(XMLRPC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/src/XmlRpc)
message(${XMLRPC_SRC})

include_directories(${BASE_INCLUDE})
include_directories(${LOG_INCLUDE})
include_directories(${ROBOT_INCLUDE})
include_directories(${TCP_INCLUDE})
include_directories(${XMLRPC_INCLUDE})

file(GLOB ALL_INCLUDE ${BASE_INCLUDE}/*.h
                        ${BASE_INCLUDE}/*.hpp
                        ${LOG_INCLUDE}/*.h
                        ${ROBOT_INCLUDE}/*.h
                        ${TCP_INCLUDE}/*.h
                        ${XMLRPC_INCLUDE}/*.h)


file(GLOB ALL_SRC ${BASE_SRC}/*.cpp
                    ${LOG_SRC}/*.c
                    ${LOG_SRC}/*.cpp
                    ${ROBOT_SRC}/*.cpp
                    ${TCP_SRC}/*.cpp
                    ${XMLRPC_SRC}/*.cpp)



if(WIN32)
    # 静态库路径
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)
    # 动态库路径
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

    add_library(fairino SHARED ${ALL_INCLUDE} ${ALL_SRC})
    #add_library(fairinod STATIC ${ALL_INCLUDE} ${ALL_SRC})
    target_link_libraries(fairino ws2_32)
    target_link_libraries(fairino Shlwapi)
        
    add_executable(test_fairino ${ALL_INCLUDE} ${ALL_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
    target_link_libraries(test_fairino ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/Release/fairino.lib)
elseif(UNIX)
    # 静态库路径
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/LinuxBuild/bin)
    # 动态库路径
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/LinuxBuild/bin)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/LinuxBuild/bin)
    add_library(fairino SHARED ${ALL_INCLUDE} ${ALL_SRC})
    # 添加调试信息
    SET(CMAKE_BUILD_TYPE "Release")  
    target_compile_options(fairino PRIVATE -g)
    # 关闭优化选项
    target_compile_options(fairino PRIVATE -O0)
    # 为动态库生成版本信息
    set_target_properties(fairino PROPERTIES VERSION 2.1.4 SOVERSION 2)

    add_executable(test_fairino ${ALL_INCLUDE} ${ALL_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
    target_link_libraries(test_fairino pthread)
    SET(CMAKE_BUILD_TYPE "Release")
    # 添加调试信息
    target_compile_options(test_fairino PRIVATE -g)
    # 关闭优化选项
    target_compile_options(test_fairino PRIVATE -O0)
    target_link_libraries(test_fairino ${CMAKE_CURRENT_SOURCE_DIR}/LinuxBuild/bin/libfairino.so)
endif()