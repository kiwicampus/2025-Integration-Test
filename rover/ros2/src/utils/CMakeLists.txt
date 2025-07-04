cmake_minimum_required(VERSION 3.5)
project(utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -fPIC")

set(DEPENDENCIES
  rclcpp
)

include_directories(include)

# Adding libraries
add_library(utils 
  lib/console.cpp
  lib/string_utils.cpp
)

target_link_libraries(utils ${rclcpp_LIBRARIES})

target_include_directories(utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(gcs_test lib/gcp_storage.cpp)
# target_link_libraries(gcs_test google-cloud-cpp::storage ${OpenCV_LIBS})

ament_target_dependencies(utils ${DEPENDENCIES})

ament_export_dependencies(rclcpp)
ament_export_targets(export_utils HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS utils
  EXPORT export_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()