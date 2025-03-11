# cmake_minimum_required(VERSION 3.8)
# project(vehicle_interface)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# #Eigen
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)


# # extra library 
# find_package(project_utils REQUIRED)



# # hpipm
# # find_library(HPIPM_LIB hpipm PATHS /hpipm/lib REQUIRED)
# # find_library(BLASFEO_LIB blasfeo PATHS /blasfeo/lib REQUIRED)

# # set(HPIPM_INCLUDE_DIR "/hpipm/include")
# # set(HPIPM_LIBRARY_DIR "/hpipm/lib")
# # # include_directories(${HPIPM_INCLUDE_DIR})
# # # link_directories(${HPIPM_LIBRARY_DIR})
# # # blasfeo
# # set(BLASFEO_INCLUDE_DIR "/blasfeo/include")
# # set(BLASFEO_LIBRARY_DIR "/blasfeo/lib")
# # include_directories(${BLASFEO_INCLUDE_DIR})
# # link_directories(${BLASFEO_LIBRARY_DIR})
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# add_executable(vehicle_node src/vehicle_interface_node.cpp)
# ament_target_dependencies(vehicle_node rclcpp Eigen3)
# target_link_libraries(vehicle_node project_utils)


# add_executable(mpc_node src/mpc_node.cpp)
# ament_target_dependencies(mpc_node rclcpp std_msgs)
# target_include_directories(mpc_node PRIVATE /hpipm/include /blasfeo/include)
# # target_include_directories(mpc_node PRIVATE ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
# target_link_libraries(mpc_node Eigen3::Eigen /hpipm/lib/libhpipm.so /blasfeo/lib/libblasfeo.so -lm )

# # Link the extra static library
# target_link_libraries(mpc_node project_utils)


# install(TARGETS
#   mpc_node
#   vehicle_node
#   DESTINATION lib/${PROJECT_NAME}
# )



cmake_minimum_required(VERSION 3.8)
project(vehicle_interface)

# Enable strict compiler warnings for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Eigen packages
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Extra library (compiled functions from project_utils_lib)
find_package(project_utils REQUIRED)

# Find HPIPM and BLASFEO
# find_path(HPIPM_INCLUDE_DIR NAMES hpipm.h PATHS /hpipm/include)
# find_library(HPIPM_LIB NAMES hpipm PATHS /hpipm/lib)
# find_path(BLASFEO_INCLUDE_DIR NAMES blasfeo.h PATHS /blasfeo/include)
# find_library(BLASFEO_LIB NAMES blasfeo PATHS /blasfeo/lib)


# target_include_directories(mpc_node PRIVATE /hpipm/include /blasfeo/include)
# # target_include_directories(mpc_node PRIVATE ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
# target_link_libraries(mpc_node Eigen3::Eigen /hpipm/lib/libhpipm.so /blasfeo/lib/libblasfeo.so -lm )

# Include HPIPM and BLASFEO headers if found
# if(HPIPM_LIB AND BLASFEO_LIB)
#   message(STATUS "HPIPM found: ${HPIPM_LIB}")
#   message(STATUS "BLASFEO found: ${BLASFEO_LIB}")
#   include_directories(${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
# else()
#   message(WARNING "HPIPM or BLASFEO not found.")
# endif()

# # Add vehicle_node executable
# add_executable(vehicle_node src/vehicle_interface_node.cpp)

# # # target_include_directories(vehicle_node PRIVATE ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
# # # target_link_libraries(vehicle_node Eigen3::Eigen ${HPIPM_LIB} ${BLASFEO_LIB} -lm)

# # target_include_directories(vehicle_node PRIVATE /hpipm/include /blasfeo/include)
# # target_link_libraries(vehicle_node Eigen3::Eigen /hpipm/lib/libhpipm.so /blasfeo/lib/libblasfeo.so -lm )


# ament_target_dependencies(vehicle_node rclcpp std_msgs Eigen3 project_utils)
# target_link_libraries(vehicle_node /root/workspace/install/project_utils/lib/libproject_utils_lib.a)
# target_link_libraries(vehicle_node project_utils_lib)

# Add mpc_node executable
add_executable(mpc_node src/mpc_node.cpp)
# target_include_directories(mpc_node PRIVATE ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
# target_link_libraries(mpc_node Eigen3::Eigen ${HPIPM_LIB} ${BLASFEO_LIB} -lm)
target_include_directories(mpc_node PRIVATE /hpipm/include /blasfeo/include)
target_link_libraries(mpc_node Eigen3::Eigen /hpipm/lib/libhpipm.so /blasfeo/lib/libblasfeo.so -lm )

ament_target_dependencies(mpc_node rclcpp std_msgs Eigen3 project_utils)
#target_link_libraries(mpc_node project_utils_lib)
#option1
target_link_libraries(mpc_node /root/workspace/install/project_utils/lib/libproject_utils_lib.a)


# Installation rules
install(TARGETS
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

