#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

class OdometryBridge : public rclcpp::Node
{
    public:
        OdometryBridge() : Node("odometry_bridge_node")
        {
            rclcpp::QoS px4_qos_profile(10);
            px4_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            px4_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

            px4_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry",px4_qos_profile,
                std::bind(&OdometryBridge::odometry_callback, this, std::placeholders::_1));

            topic_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(100).keep_last(100));

            RCLCPP_INFO(this->get_logger(), "Odometry Bridge Started");
        }
    private:
        void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
        {
            nav_msgs::msg::Odometry odom_msg;

            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = msg->position[0];
            odom_msg.pose.pose.position.y = msg->position[1];
            odom_msg.pose.pose.position.z = msg->position[2];

            odom_msg.pose.pose.orientation.x = msg->q[1];
            odom_msg.pose.pose.orientation.y = msg->q[2];
            odom_msg.pose.pose.orientation.z = msg->q[3];
            odom_msg.pose.pose.orientation.w = msg->q[0];

            odom_msg.twist.twist.linear.x = msg->velocity[0];
            odom_msg.twist.twist.linear.y = msg->velocity[1];
            odom_msg.twist.twist.linear.z = msg->velocity[2];

            topic_publisher_->publish(odom_msg);
        }

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topic_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<OdometryBridge>());
    rclcpp::shutdown();
    return 0;
}