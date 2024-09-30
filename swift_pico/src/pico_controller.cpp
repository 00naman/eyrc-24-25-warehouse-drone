#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "pid_msg/msg/pid_tune.hpp"
#include "pid_msg/msg/pid_error.hpp"
#include "rclcpp/rclcpp.hpp"

#define MAX_INTEGRAL_ERROR 1e4
#define PIPE(x) ( ((x) < 1000) ? 1000 : (((x) > 2000) ? 2000 : (x)) )

using std::placeholders::_1;
using namespace std::chrono_literals;

struct pid_holder {
    double kp;
    double ki;
    double kd;
};

struct error_holder {
    double prev_error;
    double total_error;
};

class Swift_Pico : public rclcpp::Node {
public:
    Swift_Pico() : Node("pico_controller"), got_setpoint_wrt_drone(false), first_pid(true) {

        this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

        this->declare_parameter<double>("throttle.kp", 0.0);
        this->declare_parameter<double>("throttle.ki", 0.0);
        this->declare_parameter<double>("throttle.kd", 0.0);

        this->declare_parameter<double>("roll.kp", 0.0);
        this->declare_parameter<double>("roll.ki", 0.0);
        this->declare_parameter<double>("roll.kd", 0.0);

        this->declare_parameter<double>("pitch.kp", 0.0);
        this->declare_parameter<double>("pitch.ki", 0.0);
        this->declare_parameter<double>("pitch.kd", 0.0);

        this->throttle.kp = this->get_parameter("throttle.kp").as_double();
        this->throttle.ki = this->get_parameter("throttle.ki").as_double();
        this->throttle.kd = this->get_parameter("throttle.kd").as_double();

        this->roll.kp = this->get_parameter("roll.kp").as_double();
        this->roll.ki = this->get_parameter("roll.ki").as_double();
        this->roll.kd = this->get_parameter("roll.kd").as_double();

        this->pitch.kp = this->get_parameter("pitch.kp").as_double();
        this->pitch.ki = this->get_parameter("pitch.ki").as_double();
        this->pitch.kd = this->get_parameter("pitch.kd").as_double();

        this->declare_parameter<double>("setpoint.x", 0.0);
        this->declare_parameter<double>("setpoint.y", 0.0);
        this->declare_parameter<double>("setpoint.z", 0.0);

        this->setpoint.pose.position.x = this->get_parameter("setpoint.x").as_double();
        this->setpoint.pose.position.y = this->get_parameter("setpoint.y").as_double();
        this->setpoint.pose.position.z = this->get_parameter("setpoint.z").as_double();

        this->declare_parameter<double>("eta", 100.0);
        this->eta = this->get_parameter("eta").as_double();

        this->sample_time = 333ms; //in milli-seconds

        this->ethrottle.prev_error = 0.0;
        this->ethrottle.total_error = 0.0;
        this->eroll.prev_error = 0.0;
        this->eroll.total_error = 0.0;
        this->epitch.prev_error = 0.0;
        this->epitch.total_error = 0.0;

        this->command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
        this->pid_error_pub = this->create_publisher<pid_msg::msg::PIDError>("/pid_error", 10);

        this->arm();
        this->run_pid = this->create_wall_timer(
            this->sample_time, 
            std::bind(&Swift_Pico::pid, this));
        this->measure_setpoint_wrt_drone = this->create_wall_timer(
            10ms,
            std::bind(&Swift_Pico::measure_setpoint, this));

    }

private:

    // pid holders
    float eta;
    pid_holder throttle, roll, pitch;
    error_holder ethrottle, eroll, epitch;
    geometry_msgs::msg::PoseStamped setpoint;
    geometry_msgs::msg::PoseStamped setpoint_wrt_drone;
    bool got_setpoint_wrt_drone;
    bool first_pid;

    // rclcpp holders
    std::chrono::milliseconds sample_time;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   
    
    rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr command_pub;
    rclcpp::Publisher<pid_msg::msg::PIDError>::SharedPtr pid_error_pub; 

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr whycon_sub;
    rclcpp::TimerBase::SharedPtr run_pid;
    rclcpp::TimerBase::SharedPtr measure_setpoint_wrt_drone;

private:

    void disarm() {
        auto cmd = swift_msgs::msg::SwiftMsgs();
        cmd.rc_roll = 1000;
        cmd.rc_pitch = 1000;
        cmd.rc_yaw = 1000;
        cmd.rc_throttle = 1000;
        cmd.rc_aux4 = 1000;
        this->command_pub->publish(cmd);
    }

    void arm() {   
        auto cmd = swift_msgs::msg::SwiftMsgs();
        disarm();
        cmd.rc_roll = 1500;
        cmd.rc_pitch = 1500;
        cmd.rc_yaw = 1500;
        cmd.rc_throttle = 1500;
        cmd.rc_aux4 = 2000;
        this->command_pub->publish(cmd);

    }

    void pid() {

        if (!(this->got_setpoint_wrt_drone)) {
            return ;
        }

        std::cout << this->setpoint_wrt_drone.pose.position.x << ", ";
        std::cout << this->setpoint_wrt_drone.pose.position.y << ", ";
        std::cout << this->setpoint_wrt_drone.pose.position.z << std::endl;

        // compute velocity in x, y and z
        float vx = this->setpoint_wrt_drone.pose.position.x / this->eta;
        float vy = this->setpoint_wrt_drone.pose.position.y / this->eta;
        float vz = -this->setpoint_wrt_drone.pose.position.z / this->eta;

        this->ethrottle.total_error += vz;
        this->eroll.total_error += vy;
        this->epitch.total_error += vx;

        this->ethrottle.total_error = std::min(this->ethrottle.total_error, MAX_INTEGRAL_ERROR);
        this->eroll.total_error = std::min(this->eroll.total_error, MAX_INTEGRAL_ERROR);
        this->epitch.total_error = std::min(this->epitch.total_error, MAX_INTEGRAL_ERROR);

        if (this->first_pid) {
            
            this->ethrottle.prev_error = vz;
            this->eroll.prev_error = vy;
            this->epitch.prev_error = vx;

            this->first_pid = false;
            return ;
        }

        auto cmd = swift_msgs::msg::SwiftMsgs();
        auto error_pub = pid_msg::msg::PIDError();

        cmd.rc_throttle = 1533 + this->throttle.kp * vz + 
                                this->throttle.ki * this->ethrottle.total_error + 
                                this->throttle.kd * (vz - this->ethrottle.prev_error);
        cmd.rc_roll = 1500 + this->roll.kp * vy + 
                                this->roll.ki * this->eroll.total_error + 
                                this->roll.kd * (vy - this->eroll.prev_error);
        cmd.rc_pitch = 1500 + this->pitch.kp * vx + 
                                this->pitch.ki * this->epitch.total_error + 
                                this->pitch.kd * (vz - this->epitch.prev_error);

        cmd.rc_throttle = PIPE(cmd.rc_throttle);
        cmd.rc_roll = PIPE(cmd.rc_roll);
        cmd.rc_pitch = PIPE(cmd.rc_pitch);

        this->command_pub->publish(cmd);
        error_pub.roll_error = vy;
        error_pub.pitch_error = vx;
        error_pub.throttle_error = vz;
        this->pid_error_pub->publish(error_pub);

        this->ethrottle.prev_error = vz;
        this->eroll.prev_error = vy;
        this->epitch.prev_error = vx;
        
    }

    void measure_setpoint() {
        try { 
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "drone",  
                "world",  
                tf2::TimePointZero); 

            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(setpoint, setpoint_wrt_drone, transform_stamped);
            this->got_setpoint_wrt_drone = true;
        }catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Swift_Pico>());
    rclcpp::shutdown();
    return 0;
}