#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuBridge : public rclcpp::Node
{
    public:
        ImuBridge() : Node("Imu_bridge_node")
        {
            rclcpp::QoS px4_qos_profile(10);
            px4_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            px4_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

            px4_subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
                "/fmu/out/sensor_combined", px4_qos_profile,
                std::bind(&ImuBridge::sensor_callback, this, std::placeholders::_1));
            

            rclcpp::QoS topic_qos_profile(10);
            topic_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            topic_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            
            topic_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu",topic_qos_profile); //topic name check!

            RCLCPP_INFO(this->get_logger(), "IMU Bridge Node Started");
        }

    private:
        void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
        {
            sensor_msgs::msg::Imu imu_msg;

            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_frame"; //Imu frame set

            imu_msg.linear_acceleration.x = msg->accelerometer_m_s2[0];
            imu_msg.linear_acceleration.y = msg->accelerometer_m_s2[1];
            imu_msg.linear_acceleration.z = msg->accelerometer_m_s2[2];

            imu_msg.angular_velocity.x = msg->gyro_rad[0];
            imu_msg.angular_velocity.y = msg->gyro_rad[1];
            imu_msg.angular_velocity.z = msg->gyro_rad[2];

            topic_publisher_->publish(imu_msg);
        }

        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr topic_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuBridge>());
    rclcpp::shutdown();
    return 0;
}
