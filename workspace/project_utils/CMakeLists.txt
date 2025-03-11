cmake_minimum_required(VERSION 3.8)
project(project_utils)

# Enable strict compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Required for message dependencies

# Create a static library from source files
add_library(project_utils_lib STATIC 
  src/vehicle_class.cpp 
  src/integrator_class.cpp
)

# Specify include directories for both build and install interfaces
target_include_directories(project_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(project_utils_lib rclcpp Eigen3)

# Install the static library with export name "project_utils_export"
install(TARGETS project_utils_lib
  EXPORT project_utils_export
  ARCHIVE DESTINATION lib
)

# Install header files
install(
  DIRECTORY include/project_utils/
  DESTINATION include/project_utils
)

# Install configuration files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Define message files
set(MSG_FILES
  "msg/EigenVector.msg"
)

# Generate the message interfaces (target name is based on the project name)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

# Expose generated message headers to other packages
target_include_directories(project_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



# Export targets and dependencies using only ament_export_targets (the ament_export_interfaces call has been removed)
ament_export_targets(project_utils_export HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 rosidl_default_runtime)

# Setup testing dependencies if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks if licenses are not added yet
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
