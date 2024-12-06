cmake_minimum_required(VERSION 3.10)
project(cybergear)
 
#设定编译参数
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_COMPILER "/usr/bin/g++")



# generate a lib
 add_library(${PROJECT_NAME}
    src/cyber_gear_protocol.c
    src/float16.c
    src/utils.c
)


#target_link_libraries(${PROJECT_NAME})

 ## Include headers
 target_include_directories(${PROJECT_NAME} PUBLIC include)

