#include <rosbag_player/rosbag_player.hpp>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

TEST(rosbag_player, rosbag_player)
{
    auto rosbag_player = RosBagPlayer("gs://autonomy-vision/rosbags/test_bag/test_bag_0.mcap");

    std::filesystem::path path(rosbag_player.get_local_path());
    EXPECT_TRUE(std::filesystem::exists(path));

    rosbag_player.play();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node("dummy");
    auto nodes_list = node.get_node_names();

    EXPECT_GT(nodes_list.size(), 0);

    rosbag_player.join();
    rclcpp::shutdown();
}