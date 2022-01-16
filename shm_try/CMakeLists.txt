cmake_minimum_required(VERSION 2.8.9)
project(uart_serial_example)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Bring the headers
include_directories(include)
include_directories(${Boost_INCLUDE_DIR})

# Can manually add the sources using the set command as follows:
set(SOURCES src/uart_driver.cpp src/uart_main.cpp)

# However, the file(GLOB...) allows for wildcard additions:
# file(GLOB SOURCES "src/*.cpp")

add_executable(shm_example ${SOURCES})

# required for forcing -lpthread
target_link_libraries(shm_example ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} -lrt)
