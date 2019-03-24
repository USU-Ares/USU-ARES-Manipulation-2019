#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8MultiArray.h>

#include <vector>

#include "nyx_solver.hpp"

class ArmController {
    public:
        // Default constructor
        ArmController();
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        void applyDelta(double d_x, double d_y, double d_z);
        void moveArm();

        void spinner();

        Solver solver;

        // ROS variables
        ros::NodeHandle nh_;

        int linear_, angular_;
        double l_scale_, a_scale_;
        ros::Publisher dof_pub_;
        ros::Subscriber joy_sub_;

        std::vector<int> currentAngle;
        std::vector<int> destinationAngle;

        sensor_msgs::Joy::ConstPtr& old_commands;

        std::vector<sensor_msgs::Joy::ConstPtr&> joyMessages;
};

ArmController::ArmController():
    linear_(1),
    angular_(2),
    nh_()
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);


    // Publishers to the Arduino
    dof_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("dof", 1);

    // Initialize solver
    std::vector<Link> chain;
    chain.push_back(Link(10, 1, -90,90, 0,180));
    chain.push_back(Link(10, 2, -90,90, 0,180));
    chain.push_back(Link(10, 2, -90,90, 0,180));
    chain.push_back(Link(10, 2, -90,90, 0,180));
    Pose startPose(20, 20, 20);
    solver = Solver(chain, startPose);

    // Set current and destination angles
    for (int i=0; i<chain.size(); i++) {
        int duty = chain[i].getDuty(chain[i].getCurrentAngle());
        currentAngle.push_back( duty );
        destinationAngle.push_back( duty );
    }

    message_count = 0;

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmController::joyCallback, this);
}

void ArmController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    /*
    // Move all joy messages back one in the queue
    joyMessages.push_back(joy);
    
    // Reduce to size 5 if greater
    if (joyMessages.size() > 5) {
        joyMessages.erase(joyMessages.begin());
    }
    */

    // Get current destination
    Pose dest = solver.getTarget();
    
    // Delta multiplier
    double mult = 0.1;

    // Adjust Z destination by left joystick
    dest.deltaZ( (l_scale_*joy->axes[1]) * mult );
    // Adjust X destination by left joystick
    dest.deltaX( (l_scale_*joy->axes[0]) * mult );
    // Adjust Y destination by right joystick
    dest.deltaY( (l_scale_*joy->axes[4]) * mult );
    // Adjust X destination by right joystick
    dest.deltaX( (l_scale_*joy->axes[3]) * mult );

    // Solve for angles
    bool found = solver.updateTargetPose(dest);

    if (found) {
        std::vector<Link> chain = solver.getChain();
        for (int i=0; i<chain.size(); i++) {
            destinationAngle[i] = chain[i].getCurrentAngle();
        }
    }

    // TODO change to be multithreaded?
    moveArm();
}

void moveArm() {
    bool needToUpdate = false;
    for (int i=0; i<currentAngle.size(); i++) {
        if (currentAngle[i] != destinationAngle[i])
            needToUpdate = true;
    }

    // Update angles, probably change for speed later
    double speedMult = 1.0;
    for (int i=0; i<currentAngle.size(); i++) {
        // TODO implement
        //int delta = destinationAngle[i] - currentAngle[i];
        //currentAngle[i] = delta/speedMult;
        currentAngle[i] = destinationAngle[i];
    }

    // If any angles are different, construct message and send it
    std_msgs::UInt8MultiArray msg;
    // Set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = currentAngle.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "i";

    // Copy data into message
    msg.data.clear();
    msg.data.insert(msg.data.end(), currentAngle.begin(), currentAngle.end());

    // Publish message
    dof_pub_.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ArmController arm_controller;

    ros::spin();
}
