#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

class GpsBridge : public rclcpp::Node
{
    public:
        GpsBridge() : Node("Gps_bridge_node")
        {
            rclcpp::QoS px4_qos_profile(10);
            px4_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            px4_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

            gps_subscription_ = this->create_subscription<px4_msgs::msg::SensorGps>(
                "/fmu/out/vehicle_gps_position", px4_qos_profile,
                std::bind(&GpsBridge::gps_callback, this, std::placeholders::_1));

            navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps",10);

            RCLCPP_INFO(this->get_logger(), "PX4 GPS to NavSatFix Bridge Node Started");
        }
    private:
        void gps_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
        {
            sensor_msgs::msg::NavSatFix navsat_msg;

            navsat_msg.header.stamp = this->now();
            navsat_msg.header.frame_id = "gps_frame";

            navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

            navsat_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            navsat_msg.latitude = msg->latitude_deg; // 1e7->degree
            navsat_msg.longitude = msg->longitude_deg;
            navsat_msg.altitude = msg->altitude_msl_m; //mm -> m

            navsat_msg.position_covariance[0] = msg->eph * msg->eph;
            navsat_msg.position_covariance[4] = msg->eph * msg->eph;
            navsat_msg.position_covariance[8] = msg->epv * msg->epv;

            navsat_msg.position_covariance[1] = msg->s_variance_m_s;  // X-Y 방향 속도-위치 공분산
            navsat_msg.position_covariance[5] = msg->c_variance_rad;

            navsat_publisher_->publish(navsat_msg);
            //RCLCPP_INFO(this->get_logger(), "Published NavSatFix Data");
        }

        rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsBridge>());
    rclcpp::shutdown();
    return 0;
}