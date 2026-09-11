cmake_minimum_required(VERSION 3.0.2)
project(api_routes)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/../.deps CACHE PATH "Directory for fetched content")
include(FetchContent)

FetchContent_Declare(paho_mqtt_cpp GIT_REPOSITORY https://github.com/eclipse/paho.mqtt.cpp.git GIT_TAG v1.3.2 GIT_SHALLOW TRUE)
FetchContent_Declare(yaml-cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git GIT_TAG 0.8.0 GIT_SHALLOW TRUE)
FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
FetchContent_Declare(dk GIT_REPOSITORY https://github.com/HGGshiwo/dk.git GIT_TAG main GIT_SHALLOW TRUE)

set(PAHO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(PAHO_BUILD_SHARED ON CACHE BOOL "" FORCE)
set(PAHO_WITH_MQTT_C ON CACHE BOOL "" FORCE)
set(PAHO_ENABLE_SSL OFF CACHE BOOL "" FORCE)
set(PAHO_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(PAHO_BUILD_DEB_PACKAGE OFF CACHE BOOL "" FORCE)

set(YAML_CPP_INSTALL OFF CACHE BOOL "Disable yaml-cpp install" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Disable yaml-cpp tests" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Disable yaml-cpp tools" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Disable yaml-cpp contrib" FORCE)

FetchContent_MakeAvailable(paho_mqtt_cpp yaml-cpp nlohmann_json dk)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)

dk_create_json_library(my_json_lib "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
)

add_service_files(
  FILES
  StringSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime std_msgs geometry_msgs sensor_msgs nav_msgs mavros_msgs tf2 tf2_geometry_msgs cv_bridge
)

add_compile_definitions(USE_ROS USE_ROS1)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(api_routes_node
  src/crash_handler.cpp
  src/mavlink_telemetry.cpp
  src/abnormal_reporter.cpp
  src/image_uploader.cpp
  src/api_routes_engine.cpp
  src/api_routes_topics.cpp
  src/api_routes_node.cpp
)
add_dependencies(api_routes_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(api_routes_node
  dk
  paho-mqttpp3-static
  my_json_lib
  yaml-cpp
  ${catkin_LIBRARIES}
  ${CURL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
