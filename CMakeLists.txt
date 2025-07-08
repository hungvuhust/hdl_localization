cmake_minimum_required(VERSION 3.8)
project(hdl_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_definitions(-std=c++17)
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  # -mavx causes a lot of errors!!
  add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_CXX_FLAGS "-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  ndt_omp
  fast_gicp
  hdl_global_localization
)

foreach(DEPENDENCY ${THIS_PACKAGE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

########################
## message generation ##
########################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanMatchingStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

###########
## Build ##
###########
include_directories(include)

# hdl_localization component
add_library(hdl_localization_component SHARED
  src/hdl_localization/pose_estimator.cpp
  src/hdl_localization_node.cpp
)
target_compile_definitions(hdl_localization_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(hdl_localization_component
  ${THIS_PACKAGE_DEPENDENCIES}
)
target_link_libraries(hdl_localization_component
  ${PCL_LIBRARIES}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hdl_localization_component "${cpp_typesupport_target}")

rclcpp_components_register_nodes(hdl_localization_component "hdl_localization::HdlLocalizationNode")
set(node_plugins "${node_plugins}hdl_localization::HdlLocalizationNode;$<TARGET_FILE:hdl_localization_component>\n")

# globalmap_server component
add_library(globalmap_server_component SHARED
  src/globalmap_server_node.cpp
)
target_compile_definitions(globalmap_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(globalmap_server_component
  ${THIS_PACKAGE_DEPENDENCIES}
)
target_link_libraries(globalmap_server_component
  ${PCL_LIBRARIES}
)

rclcpp_components_register_nodes(globalmap_server_component "hdl_localization::GlobalmapServerNode")
set(node_plugins "${node_plugins}hdl_localization::GlobalmapServerNode;$<TARGET_FILE:globalmap_server_component>\n")

# Combined main executable
add_executable(hdl_localization_main
  src/hdl_localization_main.cpp
  src/hdl_localization/pose_estimator.cpp
  src/hdl_localization_node.cpp
  src/globalmap_server_node.cpp
)
ament_target_dependencies(hdl_localization_main
  ${THIS_PACKAGE_DEPENDENCIES}
)
target_link_libraries(hdl_localization_main
  ${PCL_LIBRARIES}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hdl_localization_main "${cpp_typesupport_target}")


install(TARGETS
  hdl_localization_component
  globalmap_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  hdl_localization_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Install Python scripts
install(PROGRAMS
  scripts/plot_status.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(hdl_localization_component globalmap_server_component)
ament_export_dependencies(
  ${THIS_PACKAGE_DEPENDENCIES}
)

ament_package()

