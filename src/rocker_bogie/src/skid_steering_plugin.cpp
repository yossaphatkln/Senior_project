
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
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

        // Setup odometry publisher and TF broadcaster
        this->odom_pub = this->rosNode->advertise<nav_msgs::Odometry>("/odom", 50);
        this->tf_broadcaster.reset(new tf::TransformBroadcaster());

        // Velocity subscribers for wheels
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
        gzlog << "Skid Steering Plugin loaded with odometry and TF broadcasting.\n";
    }

    // Handle incoming velocity commands for each wheel
    void OnReceived_velLeftFront(const geometry_msgs::Twist::ConstPtr& msg) { this->leftFrontVel = msg->angular.z; }
    void OnCmdVelReceived_velLeftMiddle(const geometry_msgs::Twist::ConstPtr& msg) { this->leftMiddleVel = msg->angular.z; }
    void OnCmdVelReceived_velLeftBack(const geometry_msgs::Twist::ConstPtr& msg) { this->leftBackVel = msg->angular.z; }
    void OnCmdVelReceived_velRightFront(const geometry_msgs::Twist::ConstPtr& msg) { this->rightFrontVel = msg->angular.z; }
    void OnCmdVelReceived_velRightMiddle(const geometry_msgs::Twist::ConstPtr& msg) { this->rightMiddleVel = msg->angular.z; }
    void OnCmdVelReceived_velRightBack(const geometry_msgs::Twist::ConstPtr& msg) { this->rightBackVel = msg->angular.z; }

    // Function that Gazebo calls every simulation update
    void OnUpdate() {
        // Apply velocity to each wheel joint based on the received ROS messages
        this->wheels[0]->SetVelocity(0, this->leftFrontVel);
        this->wheels[1]->SetVelocity(0, this->leftMiddleVel);
        this->wheels[2]->SetVelocity(0, this->leftBackVel);
        this->wheels[3]->SetVelocity(0, this->rightFrontVel);
        this->wheels[4]->SetVelocity(0, this->rightMiddleVel);
        this->wheels[5]->SetVelocity(0, this->rightBackVel);

        // Parameters based on your robot's dimensions
        double wheel_base = 0.4;  // Distance between the left and right wheel sets (adjust as needed)
        double wheel_radius = 0.055; // Radius of the wheels (adjust as needed)
        double dt = 0.01;  // Time step; adjust to match your actual update rate

        // Calculate average velocities for left and right wheels
        double left_avg = (this->leftFrontVel + this->leftMiddleVel + this->leftBackVel) / 3.0;
        double right_avg = (this->rightFrontVel + this->rightMiddleVel + this->rightBackVel) / 3.0;

        // Calculate linear and angular velocities based on skid steering
        double vx = (left_avg + right_avg) / 2.0;
        double vth = (right_avg - left_avg) / wheel_base;
        double dx = 0;
        double dy = 0;
        double dth = 0;
        // Check wheel direction to determine updates
        bool moving_forward = left_avg > 0 && right_avg > 0;
        bool moving_backward = left_avg < 0 && right_avg < 0;
        bool rotating_left = left_avg < 0 && right_avg > 0;
        bool rotating_right = left_avg > 0 && right_avg < 0;

        if (moving_forward) {
            dx = vx * cos(this->theta) * dt;
            dy = vx * sin(this->theta) * dt;
            dth = 0;
        } else if (moving_backward) {
            dx = vx * cos(this->theta) * dt;
            dy = vx * sin(this->theta) * dt;
            dth = 0;
        } else if (rotating_left) {
            dth = vth * dt;  // Rotate left
            dx = 0;
            dy = 0;
        } else if (rotating_right) {
            dth = -vth * dt;  // Rotate right
            dx = 0;
            dy = 0;
        }

        
        // Update position and orientation
        this->x += dx;
        this->y += dy;
        this->theta += dth;

        // Publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = this->x;
        odom_msg.pose.pose.position.y = this->y;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->theta);
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.angular.z = vth;

        this->odom_pub.publish(odom_msg);

        // Declare odom_trans as a TransformStamped object
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = this->x;
        odom_trans.transform.translation.y = this->y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(this->theta);

        // Broadcast the transform
        this->tf_broadcaster->sendTransform(odom_trans);
    }

private:
    // Gazebo components
    physics::ModelPtr model;
    std::array<physics::JointPtr, 6> wheels;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;

    // ROS communication
    ros::Publisher odom_pub;
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
    ros::Subscriber velLeftFront, velLeftMiddle, velLeftBack;
    ros::Subscriber velRightFront, velRightMiddle, velRightBack;

    // Wheel velocities
    double leftFrontVel = 0.0;
    double leftMiddleVel = 0.0;
    double leftBackVel = 0.0;
    double rightFrontVel = 0.0;
    double rightMiddleVel = 0.0;
    double rightBackVel = 0.0;

    // Odometry state
    double x = 0.0, y = 0.0, theta = 0.0;
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(SkidSteeringPlugin)

}  // namespace gazebo
