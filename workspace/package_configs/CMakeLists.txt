cmake_minimum_required(VERSION 3.8)
project(package_configs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

#add_library(package_utils SHARED src/config.cpp)  #dynamic libary

add_library(package_utils STATIC src/vehicle_class.cpp src/integrator_class.cpp)


target_include_directories(package_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(package_utils rclcpp Eigen3)

# install dynamic library
# install(TARGETS package_utils
#   EXPORT package_utils_export
#   LIBRARY DESTINATION lib
# )

# Install static library
install(TARGETS package_utils
  EXPORT package_utils_export
  ARCHIVE DESTINATION lib
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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

ament_export_targets(package_utils_export HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

ament_package()
