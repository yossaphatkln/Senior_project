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

    // Get the wheel jointsz
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

    this->velLeftFront = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_left_front", 1, &SkidSteeringPlugin::OnReceived_velLeftFront, this);
    this->velLeftMiddle = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_left_middle", 1, &SkidSteeringPlugin::OnCmdVelReceived_velLeftMiddle, this);
    this->velLeftBack = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_left_back", 1, &SkidSteeringPlugin::OnCmdVelReceived_velLeftBack, this);
    this->velRightFront = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_right_front", 1, &SkidSteeringPlugin::OnCmdVelReceived_velRightFront, this);
    this->velRightMiddle = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_right_middle", 1, &SkidSteeringPlugin::OnCmdVelReceived_velRightMiddle, this);
    this->velRightBack = this->rosNode->subscribe<geometry_msgs::Twist>(
        "/cmd_vel_right_back", 1, &SkidSteeringPlugin::OnCmdVelReceived_velRightBack, this);

    // Log success message for debugging
    gzlog << "Skid Steering Plugin loaded with separate left and right velocity topics.\n";
}

    // Handle incoming velocity commands for the left wheel
    void OnReceived_velLeftFront(const geometry_msgs::Twist::ConstPtr& msg) {
        // this->leftFrontVel = msg->angular.x;
        this->leftFrontVel = msg->angular.z;
    }

    void OnCmdVelReceived_velLeftMiddle(const geometry_msgs::Twist::ConstPtr& msg) {
        // this->leftMiddleVel = msg->linear.x;
        this->leftMiddleVel = msg->angular.z;
    }

    void OnCmdVelReceived_velLeftBack(const geometry_msgs::Twist::ConstPtr& msg) {
        // this->leftBackVel = msg->linear.x;
        this->leftBackVel = msg->angular.z;
    }

    void OnCmdVelReceived_velRightFront(const geometry_msgs::Twist::ConstPtr& msg) {
        // this->rightFrontVel = msg->linear.x;
        this->rightFrontVel = msg->angular.z;
    }

    void OnCmdVelReceived_velRightMiddle(const geometry_msgs::Twist::ConstPtr& msg) {
        // this->rightMiddleVel = msg->linear.x;
        this->rightMiddleVel = msg->angular.z;
    }

    void OnCmdVelReceived_velRightBack(const geometry_msgs::Twist::ConstPtr& msg) {
        // this->rightBackVel = msg->linear.x;
        this->rightBackVel = msg->angular.z;
    }

    // Function that Gazebo calls every simulation update
    void OnUpdate() {
        // Apply velocity to each wheel joint based on the received ROS messages
        this->wheels[0]->SetVelocity(0, this->leftFrontVel);
        this->wheels[1]->SetVelocity(0, this->leftMiddleVel);
        this->wheels[2]->SetVelocity(0, this->leftBackVel);
        this->wheels[3]->SetVelocity(0, this->rightFrontVel);
        this->wheels[4]->SetVelocity(0, this->rightMiddleVel);
        this->wheels[5]->SetVelocity(0, this->rightBackVel);
    }

private:
    // Gazebo components
    physics::ModelPtr model;
    std::array<physics::JointPtr, 6> wheels;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber velLeftFront, velLeftMiddle, velLeftBack;
    ros::Subscriber velRightFront, velRightMiddle, velRightBack;


    // Wheel velocities
    double leftFrontVel = 0.0;
    double leftMiddleVel = 0.0;
    double leftBackVel = 0.0;
    double rightFrontVel = 0.0;
    double rightMiddleVel = 0.0;
    double rightBackVel = 0.0;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SkidSteeringPlugin)

}  // namespace gazebo
