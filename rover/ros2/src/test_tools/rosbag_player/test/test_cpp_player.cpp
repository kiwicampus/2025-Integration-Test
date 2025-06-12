#include <rosbag_player/rosbag_player.hpp>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

TEST(rosbag_player, rosbag_player)
{
    try{
        auto dummy_rosbag_player = rosbag_player::RosBagPlayer("/workspace/rosbags/does_not_exist.mcap");
        FAIL() << "This should rise and exception.";
    }
    catch (std::logic_error e){
        std::cout << "Good Job\n";
    }
    auto rosbag_player = rosbag_player::RosBagPlayer("/workspace/rosbags/test_bag.mcap");

    rosbag_player::PlayOptions options;
    // TODO: populate the options
    rosbag_player.play(options);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node("dummy");
    auto nodes_list = node.get_node_names();

    EXPECT_GT(nodes_list.size(), 0);

    rosbag_player.join();
    rclcpp::shutdown();
}