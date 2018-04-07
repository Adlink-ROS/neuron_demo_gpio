cmake_minimum_required(VERSION 3.5)
project(neuron_demo_gpio)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_VERBOSE_MAKEFILE ON)
#set(CMAKE_BUILD_TYPE Debug)

# Add include directories
include_directories(${CMAKE_SOURCE_DIR}/include/neuron_gpio
                    ${CMAKE_SOURCE_DIR}/include/SEMA)

# Set link directories
link_directories(${CMAKE_SOURCE_DIR}/lib)

# Add debug definitions
if(UNIX)
    add_definitions("-DLINUX -O2 -s -Wall ")
endif(UNIX)


# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Source Code
add_executable(neuron_demo_gpio src/main.cpp src/neuronGpioNode.cpp src/neuronGpio.cpp)
target_link_libraries(neuron_demo_gpio semaeapi)
ament_target_dependencies(neuron_demo_gpio rclcpp std_msgs)

add_executable(neuron_omni_io src/omni_main.cpp src/neuronOmniIoNode.cpp src/neuronGpio.cpp)
target_link_libraries(neuron_omni_io semaeapi)
ament_target_dependencies(neuron_omni_io rclcpp std_msgs)

install(TARGETS 
neuron_demo_gpio
neuron_omni_io
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
