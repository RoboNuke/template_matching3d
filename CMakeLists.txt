cmake_minimum_required(VERSION 3.8)
project(template_matching3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pub_pcd src/view_pcd.cpp)
target_link_libraries( pub_pcd
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(pub_pcd 
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)

add_executable(ObjRecRANSAC src/RosObjRecRANSAC.cpp)
target_link_libraries( ObjRecRANSAC
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(ObjRecRANSAC 
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)

add_executable(TabletopSegmentation src/tabletop_removal.cpp)
target_link_libraries( TabletopSegmentation
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(TabletopSegmentation 
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)

add_executable(PCLRegistration src/RosPCLRegistration.cpp)
target_link_libraries( PCLRegistration
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(PCLRegistration 
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)

install(TARGETS
  pub_pcd
  ObjRecRANSAC
  TabletopSegmentation
  PCLRegistration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
