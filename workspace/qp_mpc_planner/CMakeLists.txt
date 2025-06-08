cmake_minimum_required(VERSION 3.8)
project(qp_mpc_planner)

# # 1) Force a Debug build by default (so -g is applied).
# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose build type: Debug or Release" FORCE)
# endif()

# # 2) Enable verbose make (print full compiler/linker commands).
# set(CMAKE_VERBOSE_MAKEFILE ON)

# # 3) Ensure -g -O0 is used for GCC/Clang
# if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#   add_compile_options(-g -O0)
# endif()


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    eigen3_cmake_module
    Eigen3
    project_utils
    ament_cmake
    rclcpp
    std_msgs
    eigen-quadprog
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# =============================
# Build qp_mpc_class
# =============================
add_library(qp_mpc_class
  src/qp_mpc_class.cpp
  src/get_obs_pose.cpp
  src/diff_flat_formulation.cpp
  src/bernstein_pol.cpp
  src/get_obs_pose.cpp
)
target_include_directories(qp_mpc_class PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(qp_mpc_class PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(qp_mpc_class PRIVATE
    eigen-quadprog::eigen-quadprog
    /hpipm/lib/libhpipm.so
    /blasfeo/lib/libblasfeo.so
    
)


# =============================
# Build qp_mpc_node
# =============================
add_executable(qp_mpc_planner_node src/qp_mpc_node.cpp)
target_include_directories(qp_mpc_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qp_mpc_planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(qp_mpc_planner_node qp_mpc_class)


install(TARGETS qp_mpc_planner_node
  DESTINATION lib/${PROJECT_NAME})

# =============================
# Install Launch Files and Config Files
# =============================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
