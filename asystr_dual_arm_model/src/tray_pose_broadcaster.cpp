#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// yaml-cpp
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class TrayPoseBroadcaster : public rclcpp::Node
{
public:
    TrayPoseBroadcaster()
        : Node("tray_pose_broadcaster")
    {
        // --- Load YAML file ---
        std::string pkg_share = ament_index_cpp::get_package_share_directory("asystr_dual_arm_model");
        std::string yaml_path = pkg_share + "/config/ati_setup.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);

        if (!config["frames"] || !config["frames"]["ati_tray"]) {
            RCLCPP_ERROR(this->get_logger(), "Could not find frames.ati_tray in YAML file!");
            throw std::runtime_error("Missing ati_tray config");
        }

        auto tray_cfg = config["frames"]["ati_tray"];
        std::string parent = tray_cfg["parent"].as<std::string>("base_link");
        auto xyz = tray_cfg["xyz"].as<std::vector<double>>();
        auto rpy = tray_cfg["rpy"].as<std::vector<double>>();

        // --- Initialize pose from YAML ---
        current_pose_.header.frame_id = parent;
        current_pose_.header.stamp = this->now();
        current_pose_.pose.position.x = xyz[0];
        current_pose_.pose.position.y = xyz[1];
        current_pose_.pose.position.z = xyz[2];

        tf2::Quaternion q;
        q.setRPY(rpy[0], rpy[1], rpy[2]);
        current_pose_.pose.orientation.x = q.x();
        current_pose_.pose.orientation.y = q.y();
        current_pose_.pose.orientation.z = q.z();
        current_pose_.pose.orientation.w = q.w();

        // --- Create TF broadcaster ---
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // --- Subscriber for tray pose updates ---
        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ati_setup/tray_pose", 10,
            std::bind(&TrayPoseBroadcaster::pose_callback, this, std::placeholders::_1));

        // --- Timer to continuously broadcast TF ---
        timer_ = this->create_wall_timer(
            50ms, std::bind(&TrayPoseBroadcaster::broadcast, this));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose_ = *msg;

        // Ensure timestamp is current
        current_pose_.header.stamp = this->now();

        // Default to base_link if empty
        if (current_pose_.header.frame_id.empty())
            current_pose_.header.frame_id = "base_link";
    }

    void broadcast()
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = current_pose_.header.frame_id;
            tf_msg.child_frame_id = "ati_tray";  // fixed child frame id

            tf_msg.transform.translation.x = current_pose_.pose.position.x;
            tf_msg.transform.translation.y = current_pose_.pose.position.y;
            tf_msg.transform.translation.z = current_pose_.pose.position.z;
            tf_msg.transform.rotation = current_pose_.pose.orientation;
        }

        tf_broadcaster_->sendTransform(tf_msg);
    }

    geometry_msgs::msg::PoseStamped current_pose_;
    std::mutex mutex_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrayPoseBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
