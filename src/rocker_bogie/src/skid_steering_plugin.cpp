#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace gazebo {

class SkidSteeringPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override {
    this->model = _model;

    // Get the wheel joints
    this->wheels = {
        _model->GetJoint("joint_wheel_front_left"),
        _model->GetJoint("joint_wheel_middle_left"),
        _model->GetJoint("joint_wheel_back_left"),
        _model->GetJoint("joint_wheel_front_right"),
        _model->GetJoint("joint_wheel_middle_right"),
        _model->GetJoint("joint_wheel_back_right")
    };

    // Check if wheel joints were loaded successfully
    if (wheels.empty()) {
        gzerr << "Failed to load wheel joints!\n";
        return;
    }

    // Connect to Gazebo's update event (called every simulation step)
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SkidSteeringPlugin::OnUpdate, this));

    // Ensure ROS is initialized (only once per process)
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "skid_steering_plugin", ros::init_options::NoSigintHandler);
    }

    // Create a ROS node handle
    this->rosNode.reset(new ros::NodeHandle("skid_steering_plugin"));

    // Subscribe to the /cmd_vel topic to receive velocity commands
    this->cmdVelSub = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 1, &SkidSteeringPlugin::OnCmdVelReceived, this);

    // Log success message for debugging
    gzlog << "Skid Steering Plugin loaded and ROS node initialized.\n";
    }

    // Handle incoming velocity commands from /cmd_vel topic
    void OnCmdVelReceived(const geometry_msgs::Twist::ConstPtr &msg) {
        // Extract linear and angular velocity from the message
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        // Calculate the wheel velocities for skid-steering
        leftWheelVel = linear - angular;
        rightWheelVel = linear + angular;
    }

    // Called at every simulation step to update wheel velocities
    void OnUpdate() {
        // Set velocities for the left and right wheels
        for (size_t i = 0; i < wheels.size(); ++i) {
            double velocity = (i < 3) ? leftWheelVel : rightWheelVel;  // Left wheels first 3, right wheels next 3
            wheels[i]->SetParam("fmax", 0, 10.0);  // Set max force for the joint
            wheels[i]->SetParam("vel", 0, velocity);  // Apply the calculated velocity
        }
    }

private:
    // Gazebo components
    physics::ModelPtr model;
    std::vector<physics::JointPtr> wheels;
    event::ConnectionPtr updateConnection;

    // ROS components
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber cmdVelSub;

    // Wheel velocities
    double leftWheelVel = 0.0;
    double rightWheelVel = 0.0;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SkidSteeringPlugin)

}  // namespace gazebo
