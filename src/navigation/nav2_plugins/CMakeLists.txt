cmake_minimum_required(VERSION 3.5)
project(nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pb_back_up_frees_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(layers SHARED
  src/layers/intensity_voxel_layer.cpp
)

##############
## BT Nodes ##
##############

####### Conditions #######
ament_auto_add_library(is_path_goal_reached_bt SHARED
  src/bt/condition/is_path_goal_reached.cpp)
list(APPEND bt_plugin_libs is_path_goal_reached_bt)

####### Actions #######
ament_auto_add_library(send_nav2_goal_bt SHARED
  src/bt/action/send_nav2_goal.cpp)
list(APPEND bt_plugin_libs send_nav2_goal_bt)

ament_auto_add_library(publish_nav_goal_bt SHARED
  src/bt/action/publish_nav_goal.cpp)
list(APPEND bt_plugin_libs publish_nav_goal_bt)

ament_auto_add_library(pub_spin_speed_bt SHARED
  src/bt/action/pub_spin_speed.cpp)
list(APPEND bt_plugin_libs pub_spin_speed_bt)

ament_auto_add_library(pub_twist_bt SHARED
  src/bt/action/pub_twist.cpp)
list(APPEND bt_plugin_libs pub_twist_bt)

ament_auto_add_library(select_patrol_path_bt SHARED
  src/bt/action/select_patrol_path.cpp)
list(APPEND bt_plugin_libs select_patrol_path_bt)

ament_auto_add_library(select_fixed_path_bt SHARED
  src/bt/action/select_fixed_path.cpp)
list(APPEND bt_plugin_libs select_fixed_path_bt)

ament_auto_add_library(hold_stop_flag_bt SHARED
  src/bt/action/hold_stop_flag.cpp)
list(APPEND bt_plugin_libs hold_stop_flag_bt)

ament_auto_add_library(send_nav_through_poses_bt SHARED
  src/bt/action/send_nav_through_poses.cpp)
list(APPEND bt_plugin_libs send_nav_through_poses_bt)

ament_auto_add_library(select_path_goal_pose_bt SHARED
  src/bt/action/select_path_goal_pose.cpp)
list(APPEND bt_plugin_libs select_path_goal_pose_bt)

####### Controls #######
ament_auto_add_library(recovery_node_bt SHARED
  src/bt/control/recovery_node.cpp)
list(APPEND bt_plugin_libs recovery_node_bt)

####### Decorators #######
ament_auto_add_library(rate_controller_bt SHARED
  src/bt/decorator/rate_controller.cpp)
list(APPEND bt_plugin_libs rate_controller_bt)

foreach(bt_plugin ${bt_plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

if(BUILD_TESTING)
set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

#############
## Install ##
#############

install(
  TARGETS ${bt_plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

ament_auto_package()
