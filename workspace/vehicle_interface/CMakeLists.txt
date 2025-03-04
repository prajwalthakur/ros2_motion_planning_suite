cmake_minimum_required(VERSION 3.8)
project(vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#Eigen
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# hpipm
# find_library(HPIPM_LIB hpipm PATHS /hpipm/lib REQUIRED)
# find_library(BLASFEO_LIB blasfeo PATHS /blasfeo/lib REQUIRED)

# set(HPIPM_INCLUDE_DIR "/hpipm/include")
# set(HPIPM_LIBRARY_DIR "/hpipm/lib")
# # include_directories(${HPIPM_INCLUDE_DIR})
# # link_directories(${HPIPM_LIBRARY_DIR})
# # blasfeo
# set(BLASFEO_INCLUDE_DIR "/blasfeo/include")
# set(BLASFEO_LIBRARY_DIR "/blasfeo/lib")
# include_directories(${BLASFEO_INCLUDE_DIR})
# link_directories(${BLASFEO_LIBRARY_DIR})
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

add_executable(vehicle_node src/vehicle_node.cpp)
ament_target_dependencies(mpc_node rclcpp std_msgs)
target_include_directories(mpc_node PRIVATE /hpipm/include /blasfeo/include)
# target_include_directories(mpc_node PRIVATE ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
target_link_libraries(mpc_node Eigen3::Eigen /hpipm/lib/libhpipm.so /blasfeo/lib/libblasfeo.so -lm )



install(TARGETS
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
