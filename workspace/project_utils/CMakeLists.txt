cmake_minimum_required(VERSION 3.8)
project(project_utils)

# Enable strict compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# define the package and find it in find_package
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  eigen3_cmake_module
  Eigen3
  rosidl_default_generators
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create a dynamic library from source files (.so)
add_library(project_utils_lib SHARED 
  src/vehicle_class.cpp 
  src/integrator_class.cpp
)



target_compile_features(project_utils_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Define message files
set(MSG_FILES
  "msg/EigenVector.msg"
)

# Generate the message interfaces (target name is based on the project name)
# Expose generated message headers to other packages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

# Specify include directories for both build and install interfaces
target_include_directories(project_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(project_utils_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(project_utils_lib PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})




# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install configuration files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config

)


install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  
)


# Install the dynamic library with export name "export_${PROJECT_NAME}"
install(TARGETS project_utils_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
)




# Export targets and dependencies using only ament_export_targets (the ament_export_interfaces call has been removed)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Setup testing dependencies if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks if licenses are not added yet
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
