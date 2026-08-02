cmake_minimum_required(VERSION 3.0.2)
project(path_curvature_supervisor)

# Pure C++17 library with no ROS API usage anywhere in include/ or src/.
# catkin is used only so this package builds into the workspace's shared
# build/devel space (catkin_ws/build, catkin_ws/devel) instead of a local
# build directory inside the package itself.
add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/path_preprocessor.cpp
  src/curvature_calculator.cpp
  src/curvature_smoother.cpp
  src/curve_region_detector.cpp
  src/controller_supervisor.cpp
  src/path_curvature_supervisor.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(path_curvature_supervisor_basic_usage examples/basic_usage.cpp)
target_link_libraries(path_curvature_supervisor_basic_usage ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} path_curvature_supervisor_basic_usage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_tests
    tests/test_main.cpp
    tests/test_path_preprocessor.cpp
    tests/test_curvature_calculator.cpp
    tests/test_curve_region_detector.cpp
    tests/test_controller_supervisor.cpp
  )
  if(TARGET ${PROJECT_NAME}_tests)
    target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
  endif()
endif()
