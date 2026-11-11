cmake_minimum_required(VERSION 3.8)
project(bw_fastlio_nav2)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters search kdtree)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

set(CSF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/csf/src)
add_library(csf_algorithm STATIC
  ${CSF_SOURCE_DIR}/c2cdist.cpp
  ${CSF_SOURCE_DIR}/Cloth.cpp
  ${CSF_SOURCE_DIR}/Constraint.cpp
  ${CSF_SOURCE_DIR}/CSF.cpp
  ${CSF_SOURCE_DIR}/Particle.cpp
  ${CSF_SOURCE_DIR}/point_cloud.cpp
  ${CSF_SOURCE_DIR}/Rasterization.cpp
  ${CSF_SOURCE_DIR}/XYZReader.cpp
)
target_include_directories(csf_algorithm SYSTEM PUBLIC ${CSF_SOURCE_DIR})

set(PATCHWORK_SOURCE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/patchworkpp)
add_library(patchwork_common STATIC
  ${PATCHWORK_SOURCE_DIR}/common/src/plane_fit.cpp
)
target_include_directories(patchwork_common SYSTEM PUBLIC
  ${PATCHWORK_SOURCE_DIR}/common/include
)
target_link_libraries(patchwork_common PUBLIC Eigen3::Eigen)

add_library(patchworkpp_algorithm STATIC
  ${PATCHWORK_SOURCE_DIR}/patchworkpp/src/patchworkpp.cpp
)
target_include_directories(patchworkpp_algorithm SYSTEM PUBLIC
  ${PATCHWORK_SOURCE_DIR}/patchworkpp/include
)
target_link_libraries(patchworkpp_algorithm PUBLIC
  patchwork_common Eigen3::Eigen
)

add_executable(pcd_to_nav2_map src/pcd_to_nav2_map.cpp)
target_include_directories(pcd_to_nav2_map PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_to_nav2_map csf_algorithm ${PCL_LIBRARIES})
target_compile_options(pcd_to_nav2_map PRIVATE -Wall -Wextra -Wpedantic)

add_executable(map_tuner src/map_tuner_node.cpp)
target_include_directories(map_tuner PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(map_tuner csf_algorithm ${PCL_LIBRARIES})
ament_target_dependencies(map_tuner
  rclcpp nav_msgs sensor_msgs std_msgs std_srvs pcl_conversions
)
target_compile_options(map_tuner PRIVATE -Wall -Wextra -Wpedantic)

add_executable(laser_map_ground_grid src/laser_map_ground_grid_node.cpp)
target_include_directories(laser_map_ground_grid PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(laser_map_ground_grid
  patchworkpp_algorithm ${PCL_LIBRARIES}
)
ament_target_dependencies(laser_map_ground_grid
  rclcpp nav_msgs sensor_msgs pcl_conversions
)
target_compile_options(laser_map_ground_grid PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rebuild_pgo_consistent_map src/rebuild_pgo_consistent_map.cpp)
target_include_directories(rebuild_pgo_consistent_map PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(rebuild_pgo_consistent_map ${PCL_LIBRARIES} Threads::Threads)
target_compile_options(rebuild_pgo_consistent_map PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS pcd_to_nav2_map map_tuner laser_map_ground_grid
  rebuild_pgo_consistent_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/fastlio_planar_odom.py
  scripts/navigation_startup_gate.py
  scripts/urdf_footprint.py
  scripts/nav_cmd_relay.py
  scripts/escape_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# The launch file imports urdf_footprint to derive the costmap footprint, which
# only resolves inside a symlink build; keep a plain copy in share/ as well so a
# copied install still finds it.
install(FILES scripts/urdf_footprint.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES third_party/README.md
  DESTINATION share/${PROJECT_NAME}/third_party
)

install(FILES
  third_party/csf/LICENSE.txt
  third_party/csf/UPSTREAM_COMMIT
  DESTINATION share/${PROJECT_NAME}/licenses/csf
)
install(FILES
  third_party/patchworkpp/LICENSE
  third_party/patchworkpp/UPSTREAM_COMMIT
  DESTINATION share/${PROJECT_NAME}/licenses/patchworkpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(test_nav_motion test/test_nav_motion.py)
  ament_add_pytest_test(test_fov_filter test/test_fov_filter.py)
  ament_add_gtest(test_fov_filter_geometry test/test_fov_filter_geometry.cpp)
  target_include_directories(test_fov_filter_geometry PRIVATE
    include ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_fov_filter_geometry ${PCL_LIBRARIES} Eigen3::Eigen)
endif()

ament_package()
