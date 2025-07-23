cmake_minimum_required(VERSION 3.8)
project(phine_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
# Create the plugin (also adding auxiliary functions)
add_library(gNB_plugin SHARED src/gNB_plugin.cc src/aux_functions.cc)
set_property(TARGET gNB_plugin PROPERTY CXX_STANDARD 17)

# adding gazebo gazebo-plugin and ros libraries
ament_target_dependencies(gNB_plugin ignition-plugin${IGN_PLUGIN_VER} ignition-gazebo${IGN_GAZEBO_VER} rclcpp std_msgs)

add_library(UE_plugin SHARED src/UE_plugin.cc src/aux_functions.cc)
set_property(TARGET UE_plugin PROPERTY CXX_STANDARD 17)

# adding gazebo gazebo-plugin and ros libraries
ament_target_dependencies(UE_plugin ignition-plugin${IGN_PLUGIN_VER} ignition-gazebo${IGN_GAZEBO_VER} rclcpp std_msgs)
# Install the plugin
install(TARGETS
UE_plugin gNB_plugin
  DESTINATION lib/${PROJECT_NAME})

ament_package()
