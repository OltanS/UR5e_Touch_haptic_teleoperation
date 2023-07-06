#include <iostream>
#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "ros/package.h"

#include "geometry_msgs/TwistStamped.h"
#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniButtonEvent.h"

class OmniStateToTwist
{
public:
    static const int NUM_SPINNERS = 1;
    static const int QUEUE_LENGTH = 1;
    static const std::string package_name;
    OmniStateToTwist() : spinner_(NUM_SPINNERS)
    {
        std::string teleop_config_location = ros::package::getPath(package_name) + "/config/teleop_config.yaml";
        teleop_config_ = YAML::LoadFile(teleop_config_location);
        movement_scale_ = teleop_config_["movement_scale"].as<double>();
        min_velocity_ = teleop_config_["min_velocity"].as<double>();

        ros::param::get("/omni_state/omni_name", omni_name_);
        omni_sub_ = n_.subscribe(omni_name_ + "/state", QUEUE_LENGTH, &OmniStateToTwist::omniCallback, this);
        button_sub_ = n_.subscribe(omni_name_ + "/button", QUEUE_LENGTH, &OmniStateToTwist::buttonCallback, this);
        
        std::string servo_config_location = ros::package::getPath(package_name) + "/config/servo_config.yaml";
        servo_config_ = YAML::LoadFile(servo_config_location);
        twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("servo_server/" + servo_config_["cartesian_command_in_topic"].as<std::string>(), QUEUE_LENGTH); 
        
        // Slightly hack-y, assumes the activation button is not pressed before an initial reading can be done.
        // initialized so that the program does not crash if they have not been
        last_processed_time_ = ros::Time::now();
        last_orientation_.w = 0;
        last_orientation_.x = 0;
        last_orientation_.y = 0;
        last_orientation_.z = 0;
        
        movement_active_ = false;

        spinner_.start();
        ros::waitForShutdown();
    }


private:
    void buttonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg) {
        movement_active_ = bool(msg->grey_button);
    }

    void omniCallback(const omni_msgs::OmniState::ConstPtr& msg) {
        ros::Time callbackTime = ros::Time::now();

        if (movement_active_)
        {
            geometry_msgs::TwistStamped twist;
            twist.header.stamp = callbackTime;
            double dt = (callbackTime - last_processed_time_).toSec();
            twist.twist.linear = geometry_msgs::Vector3(msg->velocity);
            twist.twist.angular = quaternionPosesToAngularVelocity(last_orientation_, msg->pose.orientation, dt);
            filterNoise(twist, min_velocity_);
            scaleTwist(twist, movement_scale_);
            transformTwistToUrFrame(twist);
            twist_pub_.publish(twist);

        }

        last_processed_time_ = callbackTime;
        last_orientation_ = msg->pose.orientation;
    }
    
    void scaleTwist(geometry_msgs::TwistStamped& twist, double scale) {
        twist.twist.linear.x *= scale;
        twist.twist.linear.y *= scale;
        twist.twist.linear.z *= scale;
        twist.twist.angular.x *= scale;
        twist.twist.angular.y *= scale;
        twist.twist.angular.z *= scale;
    }

    /*
    This transformation can be done via a 3rd party library for more complicated situations
    However, the mapping for this situation is deeply simple, and can thus be trivially done by hand
    by swapping some values around. The swaps have been determined empirically
    */
    void transformTwistToUrFrame(geometry_msgs::TwistStamped& twist) {
        double temp_x = twist.twist.linear.x;
        double temp_y = twist.twist.linear.y;

        twist.twist.linear.x = -temp_y;
        twist.twist.linear.y = temp_x;
        // Z stays as is, angular velocities are OK
    }

    // discard any velocity less than a configured minimum
    void filterNoise(geometry_msgs::TwistStamped& twist, const double& min_value) {
        if (abs(twist.twist.linear.x) < min_value) {twist.twist.linear.x = 0.0; }
        if (abs(twist.twist.linear.y) < min_value) {twist.twist.linear.y = 0.0; }
        if (abs(twist.twist.linear.z) < min_value) {twist.twist.linear.z = 0.0; }
        if (abs(twist.twist.angular.x) < min_value) {twist.twist.angular.x = 0.0; }
        if (abs(twist.twist.angular.y) < min_value) {twist.twist.angular.x = 0.0; }
        if (abs(twist.twist.angular.z) < min_value) {twist.twist.angular.x = 0.0; }
    }

    // q1 is at time t, q2 is at time t + dt, dt in seconds
    // from https://mariogc.com/post/angular-velocity-quaternions/ 
    geometry_msgs::Vector3 quaternionPosesToAngularVelocity(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2, double dt) {        
        geometry_msgs::Vector3 angular_velocity;
        angular_velocity.x = (2.0 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y);
        angular_velocity.y = (2.0 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x);
        angular_velocity.z = (2.0 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w);
        return angular_velocity;
    }

    ros::NodeHandle n_;
    ros::Subscriber omni_sub_;
    ros::Subscriber button_sub_;
    ros::Publisher twist_pub_;
    ros::AsyncSpinner spinner_;
    std::string omni_name_;
    // relates to the config required by moveit_servo
    YAML::Node servo_config_;
    // relates to the config required by this file
    YAML::Node teleop_config_;

    ros::Time last_processed_time_;
    geometry_msgs::Quaternion last_orientation_;
    bool movement_active_;

    double min_velocity_;
    double movement_scale_;
};

const std::string OmniStateToTwist::package_name = "ur_to_touch_haptic_teleoperation";
int main(int argc, char** argv)
{
    ros::init(argc, argv, OmniStateToTwist::package_name);
    OmniStateToTwist to_twist;
}
