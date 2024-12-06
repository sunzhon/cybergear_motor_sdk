cmake_minimum_required(VERSION 3.10)
project(serial)
 
# generate a lib
 add_library(${PROJECT_NAME}
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

target_sources(${PROJECT_NAME} PRIVATE
      src/impl/unix.cc
      src/impl/list_ports/list_ports_linux.cc)

target_link_libraries(${PROJECT_NAME} rt pthread)

 ## Include headers
 target_include_directories(${PROJECT_NAME} PUBLIC include)

