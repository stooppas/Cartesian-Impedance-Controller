cmake_minimum_required(VERSION 3.5)
project(cartesian_impedance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rsl
  rclcpp_components
  rclcpp_action
  parameter_traits
  control_msgs
  controller_interface
  controller_manager
  generate_parameter_library
  geometry2
  tf2_geometry_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  trajectory_msgs
  urdf
  Eigen3
  eigen3_cmake_module
)

find_package(Boost 1.74.0 REQUIRED)
find_package(PkgConfig)

pkg_check_modules(RBDyn REQUIRED RBDyn)
pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(${PROJECT_NAME}_parameters
  src/cartesian_impedance_controller_parameters.yaml
  include/${PROJECT_NAME}/validate_cic_parameters.hpp
)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/ControllerConfig.msg"
  "msg/ControllerState.msg"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_impedance_controller_ros.cpp
  src/cartesian_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/>
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${SpaceVecAlg_INCLUDE_DIRS} 
  ${RBDyn_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  "${PROJECT_NAME}_msgs__rosidl_typesupport_cpp"
  ${Boost_LIBRARIES}
  ${tinyxml2_LIBRARIES}
  RBDyn
  RBDynParsers
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} RBDyn SpaceVecAlg tinyxml2 Boost )

pluginlib_export_plugin_description_file(controller_interface cartesian_impedance_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3 rosidl_default_runtime)
ament_package()