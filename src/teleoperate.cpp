// TODO: Add as dependencies

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "yaml-cpp/yaml.h"

#include "omni_msgs/OmniState.h"
#include "omni_msgs/OmniButtonEvent.h"

class OmniStateToTwist
{
public:
    static const int NUM_SPINNERS = 1;
    static const int QUEUE_LENGTH = 1;

    OmniStateToTwist() : spinner_(NUM_SPINNERS)
    {
        ros::param::get("/omni_state/omni_name", omni_name_);
        omni_sub_ = n_.subscribe(omni_name_ + "/state", QUEUE_LENGTH, &OmniStateToTwist::omniCallback, this);
        button_sub_ = n_.subscribe(omni_name_ + "/button", QUEUE_LENGTH, &OmniStateToTwist::buttonCallback, this);
        twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("servo_server/" + config_["cartesian_command_in_topic"].as<std::string>(), QUEUE_LENGTH); 
        
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
            twist.twist.angular = quaternionPosesToAngularVelocity(last_orientation_, msg->pose.orientation, dt);
            twist.twist.linear = transformAndScaleLinearVelocities(msg->velocity);
        }
        // movement disabled
        else
        {
            twist.twist.angular.x = 0;
            twist.twist.angular.y = 0;
            twist.twist.angular.z = 0;
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
        }
        // TODO check if this copy works
        last_processed_time_ = twist.header.stamp;
        last_orientation_ = msg->pose.orientation;
        
        twist_pub_.publish(twist);
    }
    
    // q1 is at time t, q2 is at time t + dt, dt in seconds
    // from https://mariogc.com/post/angular-velocity-quaternions/ 
    geometry_msgs::Vector3 quaternionPosesToAngularVelocity(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2, double dt) {        
        geometry_msgs::Vector3 angular_velocity;
        angular_velocity.x = (2 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y);
        angular_velocity.y = (2 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y + q2.w - q1.z * q2.x);
        angular_velocity.z = (2 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w);
        return angular_velocity;
    }

    // WIP
    geometry_msgs::Vector3 transformAndScaleLinearVelocities(const geometry_msgs::Vector3& touch_velocity) {
        geometry_msgs::Vector3 rotated_velocities;
        
        // TODO: transform and scale here
        rotated_velocities.x = touch_velocity.x;
        rotated_velocities.y = touch_velocity.y;
        rotated_velocities.z = touch_velocity.z;

        return rotated_velocities;    
    }


    ros::NodeHandle n_;
    ros::Subscriber omni_sub_;
    ros::Subscriber button_sub_;
    ros::Publisher twist_pub_;
    ros::AsyncSpinner spinner_;
    std::string omni_name_;
        

    std::string config_file_ = "../config/servo_config.yaml";
    YAML::Node config_ = YAML::LoadFile(config_file_);

    ros::Time last_processed_time_;
    geometry_msgs::Quaternion last_orientation_;
    bool movement_active_;
};
