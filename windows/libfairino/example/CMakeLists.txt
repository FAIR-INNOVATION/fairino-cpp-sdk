cmake_minimum_required(VERSION 3.10.0)
project(example)

if(WIN32)
    message("this windows platform")
    add_definitions(-DWINDOWS_OPTION)
elseif(UNIX)
    message("this is unix-like system")
    add_definitions(-DLINUX_OPTION)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB SRC_FILES "./src/*.cpp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

foreach(SRC_FILE ${SRC_FILES})

    get_filename_component(EXEC_NAME ${SRC_FILE} NAME_WE)

    add_executable(${EXEC_NAME} ${SRC_FILE})

    if(WIN32)
        target_link_libraries(${EXEC_NAME} ${CMAKE_SOURCE_DIR}/lib/fairino.lib)  
    elseif(UNIX)
        SET(CMAKE_BUILD_TYPE "Debug")
        target_compile_options(${EXEC_NAME} PRIVATE -g)
        target_compile_options(${EXEC_NAME} PRIVATE -O0)
        target_link_libraries(${EXEC_NAME} ${CMAKE_SOURCE_DIR}/lib/libfairino.so)
    endif(WIN32)
    
endforeach(SRC_FILE $(SRC_FILES))