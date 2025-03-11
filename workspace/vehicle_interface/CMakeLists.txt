cmake_minimum_required(VERSION 3.8)
project(vehicle_interface)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================
# Find Dependencies
# =============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Eigen dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Extra library (project_utils)
find_package(project_utils REQUIRED)

# =============================
# Include Directories
# =============================
include_directories(
  include
)

# =============================
# Build Vehicle Interface Class
# =============================
add_library(vehicle_interface_class
  src/vehicle_interface_class.cpp
)

target_include_directories(vehicle_interface_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../project_utils/include>
)
target_link_libraries(vehicle_interface_class
  /root/workspace/install/project_utils/lib/libproject_utils_lib.a
)
ament_target_dependencies(vehicle_interface_class
  rclcpp
  Eigen3
  project_utils
)

# =============================
# Build Vehicle Node
# =============================
add_executable(vehicle_interface_node src/vehicle_interface_node.cpp)

target_link_libraries(vehicle_interface_node
  vehicle_interface_class
  /root/workspace/install/project_utils/lib/libproject_utils_lib.a
)

ament_target_dependencies(vehicle_interface_node
  rclcpp
  std_msgs
  Eigen3
  project_utils
)

# =============================
# Build MPC Node
# =============================
add_executable(mpc_node src/mpc_node.cpp)

# Include HPIPM and BLASFEO if available
target_include_directories(mpc_node PRIVATE
  /hpipm/include
  /blasfeo/include
)

# Link HPIPM and BLASFEO libraries
target_link_libraries(mpc_node
  Eigen3::Eigen
  /hpipm/lib/libhpipm.so
  /blasfeo/lib/libblasfeo.so
  -lm
  /root/workspace/install/project_utils/lib/libproject_utils_lib.a
)

ament_target_dependencies(mpc_node
  rclcpp
  std_msgs
  Eigen3
  project_utils
)

# =============================
# Install Targets
# =============================
install(TARGETS
  vehicle_interface_node
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

# =============================
# Install Launch Files and Config Files
# =============================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# =============================
# Testing Configuration
# =============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip linter warnings (if needed)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

# =============================
# Export Package
# =============================
ament_package()
