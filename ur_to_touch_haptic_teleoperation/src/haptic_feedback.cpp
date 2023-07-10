#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "ros/package.h"

#include "geometry_msgs/WrenchStamped.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniButtonEvent.h"

class URSensorToHaptic
{
public:
    static const int NUM_SPINNERS = 1;
    static const int QUEUE_LENGTH = 1;
    static const std::string package_name;
    URSensorToHaptic() : spinner_(NUM_SPINNERS) {
        std::string teleop_config_location = ros::package::getPath(package_name) + "/config/teleop_config.yaml";
        teleop_config_ = YAML::LoadFile(teleop_config_location);
        force_scale_ = teleop_config_["force_scale"].as<double>();

        wrench_sub_ = n_.subscribe("/wrench", QUEUE_LENGTH, &URSensorToHaptic::wrenchCallback, this);
        
        ros::param::get("/omni_state/omni_name", omni_name_);
        button_sub_ = n_.subscribe(omni_name_ + "/button", QUEUE_LENGTH, &URSensorToHaptic::buttonCallback, this);
        haptic_pub_ = n_.advertise<omni_msgs::OmniFeedback>(omni_name_ + "/force_feedback", QUEUE_LENGTH);

        movement_active_ = false;
        spinner_.start();
        ros::waitForShutdown();
    }


private:
    void buttonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg) {
        movement_active_ = bool(msg->grey_button);
    }

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        omni_msgs::OmniFeedback feedback_vector;
        geometry_msgs::Vector3 force_vector;
        // "Lock Position" is not considered for the application
        geometry_msgs::Vector3 pose_vector;
        // only generate feedback if movement active, otherwise we'll populate
        // the vector with default 0 initialized vectors.
        if (movement_active_) {
            force_vector = msg->wrench.force;
            transformForceToTouchFrame(force_vector);
        }

        scaleForceVector(force_vector);
        feedback_vector.force = force_vector;
        feedback_vector.position = pose_vector;
        haptic_pub_.publish(feedback_vector);
    }

    /*
````We transform from the tool frame, where the forces are measured from
    So not the same transformation as the base.
    */
    void transformForceToTouchFrame(geometry_msgs::Vector3& force_vector) {
        // Negate everything to set the direction of "reaction"
        force_vector.x *= -1;
        force_vector.z *= -1;
    }

    void scaleForceVector(geometry_msgs::Vector3& force_vector) {
        force_vector.x *= force_scale_;
        force_vector.y *= force_scale_;
        force_vector.z *= force_scale_;
    }

    ros::NodeHandle n_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber wrench_sub_;
    ros::Subscriber button_sub_;
    ros::Publisher haptic_pub_;
    std::string omni_name_;
    
    bool movement_active_;

    YAML::Node teleop_config_;
    double force_scale_;

};
    const std::string URSensorToHaptic::package_name = "ur_to_touch_haptic_teleoperation";

int main(int argc, char** argv) {
    ros::init(argc, argv, URSensorToHaptic::package_name + "_haptic");
    URSensorToHaptic to_haptic;
}
