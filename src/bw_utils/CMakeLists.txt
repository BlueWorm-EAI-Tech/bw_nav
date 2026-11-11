cmake_minimum_required(VERSION 3.8)
project(bw_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# --- Code Generation ---
# 1. Parse the YAML file
include(CMakeParseArguments)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/config/controller_indices.yaml" YAML_CONTENT)

# 2. Extract values and set them as CMake variables
string(REGEX MATCH "JOY_X_AXIS:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(JOY_X_AXIS ${CMAKE_MATCH_1})
string(REGEX MATCH "JOY_Y_AXIS:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(JOY_Y_AXIS ${CMAKE_MATCH_1})
string(REGEX MATCH "GRIPPER_POS_AXIS:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(GRIPPER_POS_AXIS ${CMAKE_MATCH_1})
string(REGEX MATCH "BUTTON_XA:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(BUTTON_XA ${CMAKE_MATCH_1})
string(REGEX MATCH "BUTTON_YB:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(BUTTON_YB ${CMAKE_MATCH_1})
string(REGEX MATCH "BUTTON_Z_MENU:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(BUTTON_Z_MENU ${CMAKE_MATCH_1})
string(REGEX MATCH "BUTTON_GRIP:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(BUTTON_GRIP ${CMAKE_MATCH_1})
string(REGEX MATCH "BUTTON_TRIGGER:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(BUTTON_TRIGGER ${CMAKE_MATCH_1})
string(REGEX MATCH "BUTTON_JOYSTICK:[ ]*([0-9]+)" _ ${YAML_CONTENT})
set(BUTTON_JOYSTICK ${CMAKE_MATCH_1})

# 3. Configure the template files to generate the final code
set(GENERATED_HPP_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}")
set(GENERATED_PY_DIR "${PROJECT_NAME}")
# 创建 Python 生成目录
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${GENERATED_PY_DIR})

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config/controller_indices.hpp.in"
  "${GENERATED_HPP_DIR}/controller_indices.hpp"
)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config/controller_indices.py.in"
  "${GENERATED_PY_DIR}/controller_indices.py"
)

# 复制 Python 工具源码到生成目录，并与构建期生成的 controller_indices.py 一起安装。
foreach(PY_MODULE IN ITEMS
    __init__.py
    common_utils.py
    robot_profile.py
    sn_publisher_node.py
    sn_utils.py
)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/bw_utils/${PY_MODULE}"
    "${CMAKE_CURRENT_BINARY_DIR}/${GENERATED_PY_DIR}/${PY_MODULE}"
    COPYONLY
  )
endforeach()

# --- Installation ---
# Install C++ header
install(
  DIRECTORY "${GENERATED_HPP_DIR}/"
  DESTINATION "include/${PROJECT_NAME}"
)
# 把 'include/' 目录下的所有内容安装到 'install/bw_utils/include/'
install(
  DIRECTORY include/
  DESTINATION include
)

# Export include directory for other C++ packages
ament_export_include_directories(include)

# ***********************************************************************************************
# 蓝虫具身笔记：
# According to [official document](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Python-Documentation.html),
# Calling rosidl_generate_interfaces and ament_python_install_package in the same
# CMake project does not work.
# See this Github issue for more info.
# It is best practice to instead separate out the message generation into a separate package.
# ***********************************************************************************************

# Install Python module
ament_python_install_package(${GENERATED_PY_DIR})

# Install Python ROS node executables
install(
  PROGRAMS
    bw_utils/sn_publisher_node.py
  RENAME sn_publisher_node
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
