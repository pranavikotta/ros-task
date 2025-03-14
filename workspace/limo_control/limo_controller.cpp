#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class LimoController : public rclcpp::Node
{
public:
    LimoController() : Node("limo_controller")
    {
        // Initialize the publisher to 'cmd_vel' topic to control robot's motion
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize the subscriber to 'odom' topic to get robot's current state
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LimoController::odom_callback, this, std::placeholders::_1));

        // Initialize the goal position/orientation
        goal_x_ = 3.0;
        goal_y_ = 3.0;
        goal_theta_ = 3.0;

        // Initialize current position
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;
    }

    // Odometry callback to update the robot's current position
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the robot's current position (x, y) and orientation (theta)
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Convert quaternion to Euler angle for orientation
        double qw = msg->pose.pose.orientation.w;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;

        // Calculate yaw (theta) from quaternion
        current_theta_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        // Compute control command to move towards the goal
        control_robot();
    }

    // Control logic to drive the robot towards the goal
    void control_robot()
    {
        // Log current position and goal position
        RCLCPP_INFO(this->get_logger(), "Current Position: x=%.2f, y=%.2f, theta=%.2f", current_x_, current_y_, current_theta_);

        // Calculate the angle to the goal
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = sqrt(dx * dx + dy * dy);
        double angle_to_goal = atan2(dy, dx);
        double angle_diff = angle_to_goal - current_theta_;

        // Normalize angle_diff to be within [-pi, pi]
        if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        // Compute proportional control gain for angular velocity
        double Kp_angular = computeKp(distance, fabs(angle_diff));

        // Stop if both angle and distance are small enough
        if (fabs(angle_diff) < 0.05 && distance < 0.05) {
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.0;  // Stop moving forward
            cmd_vel_msg.angular.z = 0.0;  // Stop rotating
            cmd_vel_pub_->publish(cmd_vel_msg);
            RCLCPP_INFO(this->get_logger(), "Target reached, stopping.");
            return;
        }

        // Compute the linear velocity using proportional controller
        geometry_msgs::msg::Twist cmd_vel_msg;
        double Kp_linear = 0.5;  // Linear gain (proportional to distance)
        cmd_vel_msg.linear.x = Kp_linear * distance;

        // Angular velocity control with gain based on distance and angle difference
        double max_angular_velocity = 0.6;
        double angular_velocity = Kp_angular * angle_diff;

        // Limits to the angular velocity
        if (angular_velocity > max_angular_velocity) {
            angular_velocity = max_angular_velocity;
        } else if (angular_velocity < -max_angular_velocity) {
            angular_velocity = -max_angular_velocity;
        }

        // Apply the angular velocity (turning)
        cmd_vel_msg.angular.z = angular_velocity;

        // Publish the control message
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    // Compute adaptive Kp for angular control based on distance and angular error
    double computeKp(double distance_error, double angular_error, 
                     double Kp_min = 0.6, double Kp_max = 1.2, 
                     double d_max = 5.0, double theta_max = 1.0)
    {
        // Scale Kp based on the distance error
        double distance_scaling = distance_error / d_max;
        
        // Scale Kp based on the angular error
        double angular_scaling = angular_error / theta_max;

        // Compute Kp as a combination of both distance and angular errors
        double Kp = Kp_min + distance_scaling * (Kp_max - Kp_min) + angular_scaling * (Kp_max - Kp_min);

        // Further adjust Kp if the angular error is large
        if (angular_error > 0.3) {
            Kp *= 1.5;
        }

        return Kp;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double current_x_;
    double current_y_;
    double current_theta_;

    double goal_x_;
    double goal_y_;
    double goal_theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoController>());
    rclcpp::shutdown();
    return 0;
}
