cmake_minimum_required(VERSION 3.10)
project(cybergear_sdk C CXX)
 
 
#设定编译参数
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_COMPILER "/usr/bin/g++")

 
#设定源码列表.cpp
#设定所有源码列表 ：aux_source_directory(<dir> <variable>)

message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

 
#设定头文件路径
include_directories("./user/include" "./serial/include"  "./cybergear/include")
#include_directories("路径1"  “路径2”...)

#设定链接库的路径（一般使用第三方非系统目录下的库）
#link_directories(../build/)
#link_directories("路径1"  “路径2”...)
 
 
#添加子目录,作用相当于进入子目录里面，展开子目录的CMakeLists.txt
#同时执行，子目录中的CMakeLists.txt一般是编译成一个库，作为一个模块
#在父目录中可以直接引用子目录生成的库
add_subdirectory(serial)
add_subdirectory(cybergear)
 
 
#生成动/静态库
#add_library(动/静态链接库名称  SHARED/STATIC(可选，默认STATIC)  源码列表)
#可以单独生成多个模块
 
add_library(${PROJECT_NAME} ./user/src/can_usb.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE cybergear serial )
target_include_directories(${PROJECT_NAME} PRIVATE serial cybergear )


#生成可执行文件S
add_executable(${PROJECT_NAME}_example  examples/main.cpp)
 
target_link_libraries(${PROJECT_NAME}_example  PRIVATE ${PROJECT_NAME})#就是g++ 编译选项中-l后的内容，不要有多余空格
target_include_directories(${PROJECT_NAME}_example PRIVATE ${PROJECT_NAME})
 
#ADD_CUSTOM_COMMAND( #执行shell命令
#          TARGET myLevelDB 
#          POST_BUILD #在目标文件myLevelDBbuild之后，执行下面的拷贝命令，还可以选择PRE_BUILD命令将会在其他依赖项执行前执行  PRE_LINK命令将会在其他依赖项执行完后执行  POST_BUILD命令将会在目标构建完后执行。
#          COMMAND cp ./myLevelDB  ../
#) 
if(CMAKE_BUILD_TYPE STREQUAL "Release")
	install(TARGETS canusb DESTINATION lib)  # This installs the library to the 'lib' directory
endif()

# Install the header and library files
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)

install(DIRECTORY user/include cybergear/include serial/include
        DESTINATION include)
