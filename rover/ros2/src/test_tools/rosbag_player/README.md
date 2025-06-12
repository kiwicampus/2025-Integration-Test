rosbag_player Package

 [rosbag_player Package](#rclpy-test-package)
  - [Responsible](#responsible)
  - [Package description](#package-description)
  - [RCLCPP usage](#rclcpp-usage)
  - [RCLPY usage](#rclpy-usage)

## Responsible
Package maintainers:

    Wilmer David Garzon Caceres 
    wilmer.garzon@kiwibot.com

## Package description

This package contains the utils to play rosbags along with your unit tests. Rosbags are downloaded from GCP, you must provide a "gs://" url.

## RCLCPP usage

Add to your package.xml

```xml
    <test_depend>rosbag_player</test_depend>
```

Add to your CMake.Lists.txt

```
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rosbag_player REQUIRED)
  # Others test dependencies

  
  set(TESTFILES
    # Your test files
  )
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

  ament_target_dependencies(${PROJECT_NAME}_test ${DEPENDENCIES} rosbag_player # test dependencies)

  install(TARGETS 
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
```

Use it in your test file, following this example:

```cpp
#include <rosbag_player/rosbag_player.hpp>
#include <gtest/gtest.h>

// More includes

TEST(test_group, test_name)
{
    // Create player object
    auto rosbag_player = RosBagPlayer("some path");

    rosbag_player.play(); // start ros bag play

    // some custom logic

    rosbag_player.join(); // wait for the rosbag end
    rclcpp::shutdown(); // End the context to end the test
}
```

## RCLPY usage

Add to your package.xml

```xml
    <test_depend>rosbag_player</test_depend>
```

Use it in your test file, following this example:

```python
from rosbag_player import RosBagPlayer
# more imports


def test_function():
    player = RosBagPlayer( "some path")
    player.play()# start ros bag play

    # some custom logic

    player.join() # wait for the rosbag end
```