#include <iostream>
#include "yaml-cpp/yaml.h"
#include "boost/circular_buffer.hpp"

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
        deadband_ = teleop_config_["deadband"].as<double>();
        consecutive_nonzero_ = teleop_config_["consecutive_nonzero"].as<int>();

        ros::param::get("/omni_state/omni_name", omni_name_);
        omni_sub_ = n_.subscribe(omni_name_ + "/state", QUEUE_LENGTH, &OmniStateToTwist::omniCallback, this);
        button_sub_ = n_.subscribe(omni_name_ + "/button", QUEUE_LENGTH, &OmniStateToTwist::buttonCallback, this);
        
        std::string servo_config_location = ros::package::getPath(package_name) + "/config/servo_config.yaml";
        servo_config_ = YAML::LoadFile(servo_config_location);
        twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("servo_server/" + servo_config_["cartesian_command_in_topic"].as<std::string>(), QUEUE_LENGTH); 
        
        previous_proposed_twists_.set_capacity(consecutive_nonzero_);

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
            deadbandFilterNoise(twist, deadband_);
            previousMovementFilterNoise(twist, consecutive_nonzero_);
            scaleTwist(twist, movement_scale_);
            transformTwistToUrFrame(twist);
            twist_pub_.publish(twist);
        }
        ROS_INFO_THROTTLE(10, "omniCallback Processing took %.5fms. Max processing time allowed is 1ms for 1000 hz.", ((ros::Time::now() - callbackTime).toSec() * 1000));
        last_processed_time_ = callbackTime;
        last_orientation_ = msg->pose.orientation;
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
    
    // discard any velocity less than a configured minimum
    void deadbandFilterNoise(geometry_msgs::TwistStamped& twist, const double& deadband) {
        if (abs(twist.twist.linear.x) < deadband) {twist.twist.linear.x = 0.0; }
        if (abs(twist.twist.linear.y) < deadband) {twist.twist.linear.y = 0.0; }
        if (abs(twist.twist.linear.z) < deadband) {twist.twist.linear.z = 0.0; }
        if (abs(twist.twist.angular.x) < deadband) {twist.twist.angular.x = 0.0; }
        if (abs(twist.twist.angular.y) < deadband) {twist.twist.angular.y = 0.0; }
        if (abs(twist.twist.angular.z) < deadband) {twist.twist.angular.z = 0.0; }
    }

    /*
    If the value for any velocity is not nonzero for any of the previous (consecutive_nonzero) messages
    set it to 0.
    This is applied after the deadband filtering.
    The previous message values are saved before setting to 0 to prevent infinite loops of just 0.
    */
    void previousMovementFilterNoise(geometry_msgs::TwistStamped& twist, const int& consecutive_nonzero) {
        geometry_msgs::TwistStamped twistCopy(twist);
        for (auto const& prev : previous_proposed_twists_) {
            if (prev.twist.linear.x == 0.0) {twist.twist.linear.x = 0.0;}
            if (prev.twist.linear.y == 0.0) {twist.twist.linear.y = 0.0;}
            if (prev.twist.linear.z == 0.0) {twist.twist.linear.z = 0.0;}
            if (prev.twist.angular.x == 0.0) {twist.twist.angular.x = 0.0;}
            if (prev.twist.angular.y == 0.0) {twist.twist.angular.y = 0.0;}
            if (prev.twist.angular.z == 0.0) {twist.twist.angular.z = 0.0;}
        }
        previous_proposed_twists_.push_back(twistCopy);
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

    boost::circular_buffer<geometry_msgs::TwistStamped> previous_proposed_twists_;

    ros::Time last_processed_time_;
    geometry_msgs::Quaternion last_orientation_;
    bool movement_active_;

    double deadband_;
    double movement_scale_;
    int consecutive_nonzero_;
};

const std::string OmniStateToTwist::package_name = "ur_to_touch_haptic_teleoperation";
int main(int argc, char** argv) {
    ros::init(argc, argv, OmniStateToTwist::package_name + "_teleop");
    OmniStateToTwist to_twist;
}
