cmake_minimum_required(VERSION 3.8)
project(vrobot_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGES_DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_geometry_msgs
)

foreach(pkg ${THIS_PACKAGES_DEPENDENCIES})
  find_package(${pkg} REQUIRED)  
endforeach()

# Set all libraries 3rd party libraries
file(GLOB 3RD_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/3rd/*/include
)


# GLOB tất cả file .so và .a trong thư mục 3rd/
file(GLOB 3RD_LIBRARIES_DIRS "3rd/*/lib/*.so" "3rd/*/lib/*.a")

# In ra danh sách thư viện tìm thấy (debug)
message(STATUS "Found libraries: ${MYLIBS}")


include_directories(
  include
)


add_executable(${PROJECT_NAME}_node
  src/vrobot_imu.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${3RD_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node 
  ${3RD_LIBRARIES_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_node 
  ${THIS_PACKAGES_DEPENDENCIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
