cmake_minimum_required(VERSION 3.10)
project(cc_mls_slam)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs geometry_msgs
  std_msgs pcl_ros pcl_conversions tf eigen_conversions visualization_msgs
  livox_ros_driver message_generation message_filters)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
# Ubuntu 20.04's classic TBB may ship no TBBConfig.cmake.
find_package(TBB QUIET CONFIG)
if(NOT TARGET TBB::tbb)
  find_path(CC_MLS_TBB_INCLUDE_DIR tbb/tbb.h)
  find_library(CC_MLS_TBB_LIBRARY NAMES tbb)
  if(NOT CC_MLS_TBB_INCLUDE_DIR OR NOT CC_MLS_TBB_LIBRARY)
    message(FATAL_ERROR "TBB missing: install libtbb-dev")
  endif()
  add_library(TBB::tbb UNKNOWN IMPORTED)
  set_target_properties(TBB::tbb PROPERTIES
    IMPORTED_LOCATION "${CC_MLS_TBB_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CC_MLS_TBB_INCLUDE_DIR}")
endif()
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(GTSAM 4.2 REQUIRED CONFIG)
if(NOT TARGET gtsam)
  message(FATAL_ERROR "GTSAM must export the gtsam target; install GTSAM 4.2 with system Eigen")
endif()

add_message_files(FILES Pose6D.msg)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES cc_mls_dynamic
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs std_msgs pcl_ros
  pcl_conversions tf eigen_conversions visualization_msgs livox_ros_driver
  message_runtime message_filters
  DEPENDS PCL OpenCV)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

# An internal library target, not a separately installed ROS package.
add_library(cc_mls_dynamic SHARED
  src/dynamic/DynObjFilter.cpp
  src/dynamic/DynObjCluster.cpp
  src/dynamic/ClusterGround.cpp
  src/dynamic/ClusterCompletion.cpp)
target_link_libraries(cc_mls_dynamic PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  ${OpenCV_LIBS} Eigen3::Eigen TBB::tbb OpenMP::OpenMP_CXX Threads::Threads)
add_dependencies(cc_mls_dynamic ${catkin_EXPORTED_TARGETS})

add_executable(cc_mls_frontend src/lio/laserMapping.cpp src/lio/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp)
target_link_libraries(cc_mls_frontend cc_mls_dynamic ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)
add_dependencies(cc_mls_frontend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
include(ProcessorCount)
ProcessorCount(CC_MLS_CPU_COUNT)
if(CC_MLS_CPU_COUNT GREATER 3)
  target_compile_definitions(cc_mls_frontend PRIVATE MP_EN MP_PROC_NUM=3)
else()
  target_compile_definitions(cc_mls_frontend PRIVATE MP_PROC_NUM=1)
endif()

add_executable(cc_mls_backend src/backend/backend.cpp src/backend/Scancontext.cpp)
target_link_libraries(cc_mls_backend ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  ${OpenCV_LIBS} Eigen3::Eigen gtsam Threads::Threads)
add_dependencies(cc_mls_backend ${catkin_EXPORTED_TARGETS})
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  target_link_libraries(cc_mls_frontend stdc++fs)
  target_link_libraries(cc_mls_backend stdc++fs)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(cc_mls_core_tests test/core_tests.cpp test/cluster_pipeline_tests.cpp
    test/reference/LegacyDynObjCluster.cpp src/lio/preprocess.cpp)
  if(TARGET cc_mls_core_tests)
    target_include_directories(cc_mls_core_tests PRIVATE src/lio)
    add_dependencies(cc_mls_core_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(cc_mls_core_tests cc_mls_dynamic)
  endif()
endif()
install(TARGETS cc_mls_dynamic cc_mls_frontend cc_mls_backend
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.cpp" EXCLUDE)
install(DIRECTORY launch config rviz_cfg licenses scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
