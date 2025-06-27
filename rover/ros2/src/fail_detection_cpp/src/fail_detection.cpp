/*! @package fail_detection
    Code Information:
        Maintainer: Eng. Pedro Alejandro Gonzalez B
        Mail: pedro@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include <fail_detection_cpp/fail_detection.hpp>
using std::placeholders::_1;

FailDetector::FailDetector(rclcpp::NodeOptions& options) : Node("fail_detection", options)
{
    auto default_sub_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS()).keep_last(1);
    auto transient_local_qos = rclcpp::QoS(1).transient_local();

    // Subscribers
    m_subs_imu_camera = this->create_subscription<sensor_msgs::msg::Imu>("/camera/imu", default_sub_qos,
                                                                         std::bind(&FailDetector::ImuCb, this, _1));
    m_subs_imu_chassis = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", default_sub_qos, std::bind(&FailDetector::ChassisImuCb, this, _1));
    m_subs_bot_speed = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry/local_odometry", default_sub_qos, std::bind(&FailDetector::BotSpeedCb, this, _1));

    // Publishers
    m_pub_fail = this->create_publisher<usr_msgs::msg::Fails>("/fail_detection/fail", transient_local_qos);

    // Timers
    // Add timers if you need someone
    // m_stop_tmr = this->create_wall_timer(std::chrono::milliseconds(300), callback_method);
    // m_stop_tmr->cancel();

    RCLCPP_INFO(this->get_logger(), "Fail detector constructor: Success!");
}

void FailDetector::ImuCb(sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Camera imu is not aligned with the robot base link frame but is published in a big rate

    // TODO:
    // Detect collitions
}

void FailDetector::ChassisImuCb(sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Chassis Imu is aligned with the robot base link frame
    // TODO:
    // Detect collitions
}

void FailDetector::BotSpeedCb(nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_bot_speed = msg->twist.twist.linear.x;
    if (m_bot_speed > 0.0f && m_motion_state != "forward")
    {
        m_motion_state = "forward";
        }
    else if (m_bot_speed < 0.0f && m_motion_state != "backwards")
    {
        m_motion_state = "backwards";
    }
}
