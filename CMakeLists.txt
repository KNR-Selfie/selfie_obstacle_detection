cmake_minimum_required(VERSION 2.8.3)
project(selfie_obstacle_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Corner.msg
  CornerArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS nodelet roscpp sensor_msgs message_runtime
  LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(CornerDetector src/CornerDetector.cpp)
add_dependencies(CornerDetector ${selfie_obstacle_detection_EXPORTED_TARGETS})

# Nodelet library
add_library(${PROJECT_NAME} src/nodelets/remove_background.cpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gmock(CornerDetectorTests test/CornerDetectorTests.cpp)
  target_link_libraries(CornerDetectorTests ${catkin_LIBRARIES} CornerDetector)
endif()
