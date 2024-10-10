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

#define MAX_INTEGRAL_ERROR 1e6
#define PIPE(x) ( ((x) < 1000) ? 1000 : (((x) > 2000) ? 2000 : (x)) )

using std::placeholders::_1;
using namespace std::chrono_literals;

struct pid_holder {
    double kp;
    double ki;
    double kd;
};

namespace change {

class Derivative {
public:
    Derivative(): prev(0.0), prev_is_there(false) {}
    Derivative(double initial_prev): prev(initial_prev), prev_is_there(true) {}

    double return_derivative(double current, double dt) {
        if (!(this->prev_is_there)) {
            this->prev = current;
            this->prev_is_there = true;
            return 0.0;
        }

        double df = (current - this->prev);
        this->prev = current;
        return df/dt;
    }

private:
    double prev;
    bool prev_is_there;
};

class Integral {
public:
    Integral(): integral(0.0) {}
    Integral(double constant): integral(constant) {}

    double return_integral(double current, double dt) {
        this->integral += current * dt;
        return this->integral;
    }

private:
    double integral;
};

}

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


        this->sample_time = 0.033; 

        this->command_pub = this->create_publisher<swift_msgs::msg::SwiftMsgs>("/drone_command", 10);
        this->pid_error_pub = this->create_publisher<pid_msg::msg::PIDError>("/pid_error", 10);

        this->arm();
        this->run_pid = this->create_wall_timer(
            33ms, 
            std::bind(&Swift_Pico::pid, this));
        this->measure_setpoint_wrt_drone = this->create_wall_timer(
            10ms,
            std::bind(&Swift_Pico::measure_setpoint, this));

    }

private:

    // pid holders
    float eta;
    pid_holder throttle, roll, pitch;
    change::Derivative dx, dy, dz;
    change::Derivative d2x, d2y, d2z;

    geometry_msgs::msg::PoseStamped setpoint;
    geometry_msgs::msg::PoseStamped setpoint_wrt_drone;
    bool got_setpoint_wrt_drone;
    bool first_pid;

    // rclcpp holders
    float sample_time;
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
        auto cmd = swift_msgs::msg::SwiftMsgs();
        auto error_pub = pid_msg::msg::PIDError();

        // computing the positional error, also the integral of the error with respect to time
        double i_error_x = this->setpoint_wrt_drone.pose.position.x;
        double i_error_y = this->setpoint_wrt_drone.pose.position.y;
        double i_error_z = -this->setpoint_wrt_drone.pose.position.z;

        // error = d(positional error) / dt 
        // error must be the first derivative of the positional error wrt time as the command is the velocity
        // compute the first derivative of the error
        double error_x = this->dx.return_derivative(i_error_x, this->sample_time);
        double error_y = this->dy.return_derivative(i_error_y, this->sample_time);
        double error_z = this->dz.return_derivative(i_error_z, this->sample_time);

        // compute the derivative of the error
        double d_error_x = this->d2x.return_derivative(error_x, this->sample_time);
        double d_error_y = this->d2y.return_derivative(error_y, this->sample_time);
        double d_error_z = this->d2z.return_derivative(error_z, this->sample_time);

        // apply the pid
        cmd.rc_pitch = 1500 + this->pitch.kp * error_x +
                            this->pitch.ki * i_error_x +
                            this->pitch.kd * d_error_x;
        cmd.rc_roll = 1500 + this->roll.kp * error_y +
                            this->roll.ki * i_error_y +
                            this->roll.kd * d_error_y;
        cmd.rc_throttle = 1500 + this->throttle.kp * error_z +
                            this->throttle.ki * i_error_z +
                            this->throttle.kd * d_error_z;
        
        cmd.rc_pitch = PIPE(cmd.rc_pitch);
        cmd.rc_roll = PIPE(cmd.rc_roll);
        cmd.rc_throttle = PIPE(cmd.rc_throttle);

        error_pub.pitch_error = error_x;
        error_pub.roll_error = error_y;
        error_pub.throttle_error = error_z;

        this->command_pub->publish(cmd);
        this->pid_error_pub->publish(error_pub);
    }

    void measure_setpoint() {
        try { 
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "drone",  
                "world",  
                tf2::TimePointZero); 

            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(this->setpoint, this->setpoint_wrt_drone, transform_stamped);
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