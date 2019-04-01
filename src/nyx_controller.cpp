#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>

#include <iostream>

#include <vector>

#include "../include/nyx_solver.hpp"

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
        ros::Subscriber joy_sub_;
        //ros::Publisher dof_pub_;
        ros::Publisher dof0_pub_;
        ros::Publisher dof1_pub_;
        ros::Publisher dof2_pub_;
        ros::Publisher dof3_pub_;

        std::vector<int> currentAngle;
        std::vector<int> destinationAngle;

        //std::vector<sensor_msgs::Joy::ConstPtr&> joyMessages;
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
    //dof_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("dof", 1);
    dof0_pub_ = nh_.advertise<std_msgs::UInt8>("dof0", 1);
    dof1_pub_ = nh_.advertise<std_msgs::UInt8>("dof1", 1);
    dof2_pub_ = nh_.advertise<std_msgs::UInt8>("dof2", 1);
    dof3_pub_ = nh_.advertise<std_msgs::UInt8>("dof3", 1);

    // Initialize solver
    std::vector<Link> chain;
    // DOF 0
    chain.push_back(Link(6.16, 1, 0,170, 50,120));
    // Vertical offset
    chain.push_back(Link(6.38, 2, 90,90, 0,0));
    // DOF 1
    chain.push_back(Link(17.63, 2, -94,9, 57,140));
    // DOF 2
    chain.push_back(Link(16.12, 2, -36,-158, 52,115));
    // DOF 3
    chain.push_back(Link(8.9, 2, -45,47, 95,134));

    // Set all angles to be in the middle (DEBUG)
    for (int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    solver = Solver(chain);

    // Set current and destination angles
    for (int i=0; i<chain.size(); i++) {

        int duty = chain[i].getDuty(chain[i].getCurrentAngle());
        currentAngle.push_back( duty );
        destinationAngle.push_back( duty );
    }

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
    double mult = 0.05;

    // Adjust Z destination by left joystick
    dest.deltaZ( (l_scale_*joy->axes[1]) * mult );
    // Adjust X destination by left joystick
    dest.deltaX( (l_scale_*joy->axes[0]) * mult );
    // Adjust Y destination by right joystick
    dest.deltaY( (l_scale_*joy->axes[4]) * mult );
    // Adjust X destination by right joystick
    dest.deltaX( (l_scale_*joy->axes[3]) * mult );

    // Solve for angles
    //bool found = solver.updateTargetPose(dest);
    State found = solver.updateTargetPose(dest);

    std::vector<Link> chain = solver.getChain();
    for (int i=0; i<chain.size(); i++) {
        destinationAngle[i] = chain[i].getCurrentAngle();
    }

    // TODO change to be multithreaded?
    moveArm();
}

void ArmController::moveArm() {
    /*
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
    */
    
    // TODO temporary holding for actual duty cycles
    std::vector<int> duty_cycles;
    std::vector<Link> chain = solver.getChain();
    for (int i=0; i<chain.size(); i++) {
        if (i != 1)
            duty_cycles.push_back( chain[i].getDuty(chain[i].getCurrentAngle()) );
    }

    /*
    // If any angles are different, construct message and send it
    std_msgs::UInt8MultiArray msg;
    // Set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = duty_cycles.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "i";

    // Copy data into message
    msg.data.clear();
    msg.data.insert(msg.data.end(), duty_cycles.begin(), duty_cycles.end());

    // Publish message
    dof_pub_.publish(msg);
    */

    // Writing to 4 topics
    std_msgs::UInt8 command_0;
    std_msgs::UInt8 command_1;
    std_msgs::UInt8 command_2;
    std_msgs::UInt8 command_3;
    command_0.data = duty_cycles[0];
    command_1.data = duty_cycles[1];
    command_2.data = duty_cycles[2];
    command_3.data = duty_cycles[3];
    std::cout << "Command 2: " << duty_cycles[3] << "\n";
    dof0_pub_.publish(command_0);
    dof1_pub_.publish(command_1);
    dof2_pub_.publish(command_2);
    dof3_pub_.publish(command_3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ArmController arm_controller;

    ros::spin();
}
