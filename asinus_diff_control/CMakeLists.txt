cmake_minimum_required(VERSION 3.8)
project(asinus_diff_control)


if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  asinus_diff_control
  SHARED
  src/diff_asinus_system.cpp
)
target_compile_features(asinus_diff_control PUBLIC cxx_std_17)
target_include_directories(asinus_diff_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/asinus_diff_control>
)
ament_target_dependencies(
  asinus_diff_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface asinus_control.xml)

# INSTALL

install(TARGETS asinus_diff_control
  EXPORT export_asinus_diff_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)
#   ament_add_pytest_test(asinus_control_test_diffbot_launch test/test_diffbot_launch.py)
# endif()

## EXPORTS
ament_export_targets(export_asinus_diff_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
