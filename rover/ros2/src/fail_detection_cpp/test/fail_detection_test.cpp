#include <gtest/gtest.h>
#include <fail_detection_cpp/fail_detection.hpp>

#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "cmr_tests_utils/multi_thread_spinner.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"

#include "rosbag_player/rosbag_player.hpp"

bool verify_fail(usr_msgs::msg::Fails fails_msg, std::string fail_event)
{
    for (auto const& fail : fails_msg.fails)
    {
        if (fail.event == fail_event) return true;
    }
    return false;
}

TEST(Fail_detection, test_rollover)
{
    setenv("ROS_DOMAIN_ID", "2", 1);
    auto rosbag_player = rosbag_player::RosBagPlayer("TODO: your bag path");

    rclcpp::init(0, nullptr);
    auto transient_local_qos = rclcpp::QoS(1).transient_local();
    auto spinner = cmr_tests_utils::MultiThreadSpinner();
    rclcpp::NodeOptions options;
    auto fail_node = std::make_shared<FailDetector>(options);
    // Create a node for subscribing to bot events
    auto bot_events_test_node = std::make_shared<rclcpp::Node>("bot_events");
    auto sub_fails = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<usr_msgs::msg::Fails>>(
        "fails", "/fail_detection/fail", transient_local_qos);
    spinner.add_node(fail_node->get_node_base_interface());
    spinner.add_node(sub_fails->get_node_base_interface());
    spinner.spin_all_nodes();

    rosbag_player::PlayOptions play_options;
    // TODO: Set options
    rosbag_player.play(play_options);
    rosbag_player.join();

    bool some_rollover = verify_fail(sub_fails->get_received_msg(), usr_msgs::msg::Fail::EVENT_ROLLOVER);
    EXPECT_TRUE(some_rollover) << "Rollover event should be detected";

    // TODO: Can you test several case in a single test function?
    rclcpp::shutdown();
}