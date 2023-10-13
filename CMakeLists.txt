#  PCL version: 1.12.1
# VTKversion: 9.0

cmake_minimum_required(VERSION 3.8)
project(depth_camera_subscriber)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)   
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})




set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  EIGEN3
  PCL
  nav_msgs
)



add_executable(depth_camera_subscriber
src/depth_camera_subscriber.cpp
src/depth_camera_subscriber_node.cpp
#src/pc_process.cpp
)

target_link_libraries (depth_camera_subscriber
#depth_camera_library
${PCL_LIBRARIES})


ament_target_dependencies(depth_camera_subscriber
  pcl_conversions
  rclcpp
  sensor_msgs
  EIGEN3
  PCL
  nav_msgs
  ${dependencies}
)


install(TARGETS depth_camera_subscriber
#depth_camera_library
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY include/
        DESTINATION sinclude
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)


ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})
ament_package()










# cmake_minimum_required(VERSION 3.8)
# project(depth_camera_subscriber)

# set(CMAKE_CXX_STANDARD 20)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()



# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)
# #find_package(VTK REQUIRED)

# message(STATUS "VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")


# include_directories(
#     ${EIGEN3_INCLUDE_DIRS}
#     ${PCL_INCLUDE_DIRS}
#     #${VTK_INCLUDE_DIRS}
#     include
# )


# link_directories(${PCL_LIBRARY_DIRS})
# #link_directories(${VTK_LIBRARY_DIRS})

# #set(ament_cmake_copyright_FOUND TRUE)
# add_library(depth_camera_library src/depth_camera_subscriber.cpp 
# #src/pc_process.cpp
# )

# add_definitions(${PCL_DEFINITIONS})

# # Create an executable
# add_executable(depth_camera_subscriber
#   src/depth_camera_subscriber_node.cpp
#   #src/pc_process.cpp
# )


# # Link Libraries
# target_link_libraries(depth_camera_subscriber
# ${PCL_LIBRARIES}
# #${VTK_LIBRARIES}
# depth_camera_library
# )


# ament_target_dependencies(depth_camera_library
# rclcpp
# sensor_msgs
# pcl_conversions
# PCL
# Eigen3
# )


# # Install targets
# install(TARGETS
# depth_camera_subscriber
# depth_camera_library
# DESTINATION lib/${PROJECT_NAME})


# # Install headers
# install(DIRECTORY include/
# DESTINATION include/)



# ament_package()

