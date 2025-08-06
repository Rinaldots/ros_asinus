cmake_minimum_required(VERSION 3.16)
project(asinus_diff_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  parameter_traits
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(diff_asinus_system SHARED
  src/diff_drive_controller.cpp
)
target_compile_features(diff_asinus_system PUBLIC cxx_std_17)
target_include_directories(diff_asinus_system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/asinus_diff_control>
)
ament_target_dependencies(diff_asinus_system PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(diff_asinus_system PRIVATE "ASINUS_DIFF_CONTROL_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface asinus_control.xml)

install(
  DIRECTORY include/
  DESTINATION include/asinus_diff_control
)
install(TARGETS diff_asinus_system
  EXPORT export_asinus_diff_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install plugin description file
install(FILES asinus_control.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch, config files
install(
  DIRECTORY
    include
  DESTINATION include/diff_drive_controller
)

ament_export_targets(export_asinus_diff_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
