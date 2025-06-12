cmake_minimum_required(VERSION 3.8)
project(rosbag_player)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(utils REQUIRED)

if(BUILD_TESTING)

  set(DEPENDENCIES
    rclcpp
    utils
  )

  # Include C++ headers
  include_directories(include)

  # Declare a C++ library
  add_library(${PROJECT_NAME}
    src/rosbag_player.cpp
  )

  target_link_libraries(${PROJECT_NAME} utils::utils)

  target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )


  ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
  ament_export_dependencies(${DEPENDENCIES})
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

  # Install C++ library
  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}/
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  # Install Python module
  ament_python_install_package(rosbag_player)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cpp_player test/test_cpp_player.cpp)
  target_link_libraries(test_cpp_player ${PROJECT_NAME})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_python_player test/test_python_player.py)

endif()

ament_package()