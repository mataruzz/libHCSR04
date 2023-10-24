cmake_minimum_required(VERSION 3.8)
project(lib_hcsr04_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include the headers from libHCSR04
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libHCSR04/include)
# and lib_hcsr04_ros
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create an executable for your wrapper
add_executable(RunUltrasonicHCSR04Wrapper src/RunUltrasonicHCSR04Wrapper.cpp)
ament_target_dependencies(RunUltrasonicHCSR04Wrapper 
                            rclcpp
                            sensor_msgs)

# Link your wrapper executable with the source file from libHCSR04
target_sources(RunUltrasonicHCSR04Wrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../libHCSR04/src/HCSR04_sensor.cpp
                                                  src/UltrasonicHCSR04Wrapper.cpp)


# Link your wrapper with the system library (wiringPi)
target_link_libraries(RunUltrasonicHCSR04Wrapper
  wiringPi
)

# Install your executable
install(TARGETS RunUltrasonicHCSR04Wrapper
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
