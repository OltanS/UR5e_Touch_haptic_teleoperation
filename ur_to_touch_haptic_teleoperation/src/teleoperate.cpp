// TODO: Add as dependencies

#include <iostream>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/TwistStamped.h"
#include "yaml-cpp/yaml.h"

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
        orientation_epsilon_ = teleop_config_["orientation_epsilon"].as<double>();
        velocity_epsilon_ = teleop_config_["velocity_epsilon"].as<double>();
        precision_ = teleop_config_["precision"].as<int>();
        movement_scale_ = teleop_config_["movement_scale"].as<double>();

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
        geometry_msgs::TwistStamped twist;
        twist.header.stamp = ros::Time::now();

        if (movement_active_)
        {
            double dt = (twist.header.stamp - last_processed_time_).toSec();
            twist.twist.linear = transformAndScaleLinearVelocities(msg->velocity);
            twist.twist.angular = quaternionPosesToAngularVelocity(last_orientation_, msg->pose.orientation, dt);
            roundTwistToPrecision(twist, precision_);
            scaleTwist(twist, movement_scale_);
            transformTwistToUrFrame(twist);
            twist_pub_.publish(twist);
        }
        // movement disabled
        // TODO check if this copy works
        last_processed_time_ = twist.header.stamp;
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
        // Z stays as is
    }

    // round to given precision to combat noise
    void roundTwistToPrecision(geometry_msgs::TwistStamped& twist, const double& precision) {
        double multiplier = pow(10, precision);
        twist.twist.angular.x = std::round(twist.twist.angular.x * multiplier) / multiplier;
        twist.twist.angular.y = std::round(twist.twist.angular.y * multiplier) / multiplier;
        twist.twist.angular.z = std::round(twist.twist.angular.z * multiplier) / multiplier;
        twist.twist.linear.x = std::round(twist.twist.linear.x * multiplier) / multiplier;
        twist.twist.linear.y = std::round(twist.twist.linear.y * multiplier) / multiplier;
        twist.twist.linear.z = std::round(twist.twist.linear.z * multiplier) / multiplier;
    }

    // q1 is at time t, q2 is at time t + dt, dt in seconds
    // from https://mariogc.com/post/angular-velocity-quaternions/ 
    geometry_msgs::Vector3 quaternionPosesToAngularVelocity(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2, double dt) {        
        // EXPERIMENTAL: Account for encoder error, if the new data's difference to old in tiny, take it the same as old.
        geometry_msgs::Quaternion q2_rounded;
        q2_rounded.x = (abs(q1.x - q2.x) < orientation_epsilon_) ? q1.x : q2.x;
        q2_rounded.y = (abs(q1.y - q2.y) < orientation_epsilon_) ? q1.y : q2.y;
        q2_rounded.z = (abs(q1.z - q2.z) < orientation_epsilon_) ? q1.z : q2.z;
        q2_rounded.w = (abs(q1.w - q2.w) < orientation_epsilon_) ? q1.w : q2.w;

        geometry_msgs::Vector3 angular_velocity;
        angular_velocity.x = (2.0 / dt) * (q1.w * q2_rounded.x - q1.x * q2_rounded.w - q1.y * q2_rounded.z + q1.z * q2_rounded.y);
        angular_velocity.y = (2.0 / dt) * (q1.w * q2_rounded.y + q1.x * q2_rounded.z - q1.y * q2_rounded.w - q1.z * q2_rounded.x);
        angular_velocity.z = (2.0 / dt) * (q1.w * q2_rounded.z - q1.x * q2_rounded.y + q1.y * q2_rounded.x - q1.z * q2_rounded.w);
        


        return angular_velocity;
    }

    // WIP
    geometry_msgs::Vector3 transformAndScaleLinearVelocities(const geometry_msgs::Vector3& touch_velocity) {
        geometry_msgs::Vector3 rotated_velocities;
        // TODO: transform and scale here
        
        // Experimental rounding, if Touch velocity less than a tiny number, set it to 0.
        rotated_velocities.x = (abs(touch_velocity.x) < velocity_epsilon_) ? 0.0 : touch_velocity.x;
        rotated_velocities.y = (abs(touch_velocity.y) < velocity_epsilon_) ? 0.0 : touch_velocity.y;
        rotated_velocities.z = (abs(touch_velocity.z) < velocity_epsilon_) ? 0.0 : touch_velocity.z;

        return rotated_velocities;    
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

    //TODO: pull these from a config instead
    // Calculated empirically by observing the change in values with no movement
    double orientation_epsilon_;
    double velocity_epsilon_;
    // Means that 2 values after the decimal are taken, to combat noise
    int precision_;
    // allow for precise movement
    double movement_scale_;

};


const std::string OmniStateToTwist::package_name = "ur_to_touch_haptic_teleoperation";
int main(int argc, char** argv)
{
    ros::init(argc, argv, OmniStateToTwist::package_name);
    OmniStateToTwist to_twist;
}