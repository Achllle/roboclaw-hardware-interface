cmake_minimum_required(VERSION 2.8.3)
project(roboclaw-hardware-interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  libroboclaw
  roscpp
)

## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES roboclaw-hardware-interface
   CATKIN_DEPENDS hardware_interface roscpp libroboclaw
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
