// tf_broadcaster.cpp

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TFBroadcaster : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr whycon_subscriber;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped;

    // control 
    bool got_poses;

public:
    TFBroadcaster(): Node("drone_tf_broadcaster"), got_poses(false) {
        this->tf_broadcaster_ = 
            std::make_shared<tf2_ros::TransformBroadcaster>(this);
        this->whycon_subscriber = 
            this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/whycon/poses", 10, 
                std::bind(&TFBroadcaster::whycon_callback, this, _1));
        this->timer_ = this->create_wall_timer(
            10ms, std::bind(&TFBroadcaster::broadcast_transform, this));

    }

private:

    void whycon_callback(const geometry_msgs::msg::PoseArray& msg) {
        this->transform_stamped.header.frame_id = "world"; 
        this->transform_stamped.child_frame_id = "drone"; 

        this->transform_stamped.transform.translation.x = msg.poses[0].position.x;
        this->transform_stamped.transform.translation.y = msg.poses[0].position.y;
        this->transform_stamped.transform.translation.z = msg.poses[0].position.z;

        this->transform_stamped.transform.rotation.x = msg.poses[0].orientation.x;
        this->transform_stamped.transform.rotation.y = msg.poses[0].orientation.y;
        this->transform_stamped.transform.rotation.z = msg.poses[0].orientation.z;
        this->transform_stamped.transform.rotation.w = msg.poses[0].orientation.w;
        this->got_poses = true;
    }

    void broadcast_transform() {
        if (!(this->got_poses)) {
            return ;
        }

        this->transform_stamped.header.stamp = this->get_clock()->now();
        tf_broadcaster_->sendTransform(this->transform_stamped);
    }

    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
