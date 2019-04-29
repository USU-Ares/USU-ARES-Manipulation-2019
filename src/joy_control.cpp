#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>

#define MIN_DOF0 50
#define MAX_DOF0 120
#define MIN_DOF1 57
#define MAX_DOF1 140
#define MIN_DOF2 52
#define MAX_DOF2 115
#define MIN_DOF3 95
#define MAX_DOF3 135
#define MIN_DOF4 65
#define MAX_DOF4 160
#define MIN_DOF5 65
#define MAX_DOF5 160

class ArmController
{
    public:
        ArmController();

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        void spinner();

        void constrainJoint(uint16_t& angle, uint16_t low, uint16_t high);
        bool constrainJoint(double& angle, uint16_t low, uint16_t high);

        ros::NodeHandle nh_;

        int linear_, angular_;
        double l_scale_, a_scale_;
        ros::Publisher dof_pub_;
        /*
        ros::Publisher dof0_pub_;
        ros::Publisher dof1_pub_;
        ros::Publisher dof2_pub_;
        ros::Publisher dof3_pub_;
        ros::Publisher dof4_pub_;
        ros::Publisher dof5_pub_;
        */
        ros::Subscriber joy_sub_;

        // Position data
        double angle0;
        double angle1;
        double angle2;
        double angle3;
        double angle4;
        double angle5;

        // Last button data
        bool lastButtons[6];

        // Attempt at coalescence
        uint8_t message_count;
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
    /*
    dof0_pub_ = nh_.advertise<std_msgs::UInt16>("dof0", 1);
    dof1_pub_ = nh_.advertise<std_msgs::UInt16>("dof1", 1);
    dof2_pub_ = nh_.advertise<std_msgs::UInt16>("dof2", 1);
    dof3_pub_ = nh_.advertise<std_msgs::UInt16>("dof3", 1);
    dof4_pub_ = nh_.advertise<std_msgs::UInt16>("dof4", 1);
    dof5_pub_ = nh_.advertise<std_msgs::UInt16>("dof5", 1);
    */
    // Publisher to the serializer
    dof_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("manipulation_data", 1);

    // Set all of the angles to valid angles, but are the same for testing
    angle0 = 100;
    angle1 = 100;
    angle2 = 100;
    angle3 = 90;
    angle4 = 100;
    angle5 = 100;

    message_count = 0;

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmController::joyCallback, this);
}

void ArmController::constrainJoint(uint16_t& angle, uint16_t low, uint16_t high) {
    if (angle < low) {
        angle = low;
    } else if (angle > high) {
        angle = high;
    }
}
bool ArmController::constrainJoint(double& angle, uint16_t low, uint16_t high) {
    if (angle < low) {
        angle = low;
        return true;
    } else if (angle > high) {
        angle = high;
        return true;
    }
    return false;
}

void ArmController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double delta;

    // Load button information
    lastButtons[0] = l_scale_*joy->buttons[0];
    lastButtons[1] = l_scale_*joy->buttons[1];
    lastButtons[2] = l_scale_*joy->buttons[2];
    lastButtons[3] = l_scale_*joy->buttons[3];
    lastButtons[4] = l_scale_*joy->buttons[4];
    lastButtons[5] = l_scale_*joy->buttons[5];

    // For main part of arm, check if at bound and trying to move in that direction
    // If so, prevent movement in that plane
    delta = l_scale_*joy->axes[1];
    if ( (angle1 <= MIN_DOF1 && delta > 0) ||
         (angle1 >= MAX_DOF1 && delta < 0) ||
         (angle2 <= MIN_DOF2 && delta > 0) ||
         (angle2 >= MAX_DOF2 && delta < 0)) {

    } else {
        // Moving forward/backward
        angle1 -= delta*0.1;
        angle2 += delta*0.1;
        angle3 += delta*0.03;
    }

    // Moving sideways along cylinder
    delta = l_scale_*joy->axes[0];
    angle0 -= delta*0.06;

    // Move hand up and down based on D pad
    delta = l_scale_*joy->axes[7];
    angle3 -= delta*0.4;
    // Rotate hand based on D pad
    delta = l_scale_*joy->axes[6];
    angle4 -= delta*0.5;

    // Move arm up and down
    delta = l_scale_*joy->axes[4];
    angle1 += delta*0.1;
    angle2 += delta*0.1;
    angle3 -= delta*0.03;

    // Triggers to open/close gripper
    delta = l_scale_*joy->axes[2] - 1;
    angle5 += delta;
    delta = l_scale_*joy->axes[5] - 1;
    angle5 -= delta;
    //ROS_INFO("Angles: %f\t%f\t%f",l_scale_*joy->axes[2],l_scale_*joy->axes[5],angle5);

    // Stored positions
    // A button 0
    if (lastButtons[0]) {
        angle0 = 100;
        angle1 = 100;
        angle2 = 100;
        angle3 = 100;
    }
    // B button 1
    else if (lastButtons[1]) {
        /*angle0 = 110;
        angle1 = 84;
        angle2 = 93;
        angle3 = 97;
        angle4 = 100;
        angle5 = 70;*/
    }
    // X button 2
    else if (lastButtons[2]) {
        angle1 = 140;
        angle2 = 115;
        angle3 = 135;
    }
    // Y button 3
    else if (lastButtons[3]) {
        angle0 = 100;
        angle1 = 140;
        angle2 = 57;
        angle3 = 52;
        angle4 = 90;
    }


    //ROS_INFO("Angles: %f\t%f\t%f",angle0,angle1,angle2);

    // Constrain joints to the duty cycles that have been found
    constrainJoint(angle0, 50, 120);
    constrainJoint(angle1, 57, 140);
    constrainJoint(angle2, 52, 115);
    constrainJoint(angle3, 95, 135);
    constrainJoint(angle4, 65, 160);
    constrainJoint(angle5, 30, 160);

    // Testing to reduce the number of messages sent
    if (message_count < 2) {
        message_count++;
        return;
    }
    message_count = 0;

    // Create messages to publish
    /*
    std_msgs::UInt16 command_0,
        command_1,
        command_2,
        command_3,
        command_4,
        command_5;

    // Publish all new angles
    command_0.data = (uint16_t)angle0;
    command_1.data = (uint16_t)angle1;
    command_2.data = (uint16_t)angle2;
    command_3.data = (uint16_t)angle3;
    command_4.data = (uint16_t)angle4;
    command_5.data = (uint16_t)angle5;

    dof0_pub_.publish(command_0);
    dof1_pub_.publish(command_1);
    dof2_pub_.publish(command_2);
    dof3_pub_.publish(command_3);
    dof4_pub_.publish(command_4);
    dof5_pub_.publish(command_5);
    */

    std::vector<uint8_t> duty_cycles;
    duty_cycles.push_back(angle0);
    duty_cycles.push_back(angle1);
    duty_cycles.push_back(angle2);
    duty_cycles.push_back(angle3);
    duty_cycles.push_back(angle4);
    duty_cycles.push_back(angle5);

    // Create single message to publish
    std::msgs::UInt8MultiArray msg;
    // Set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = duty_cycles.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "manip";

    // Copy data into message
    msg.data.clear();
    msg.data.insert(msg.data.end(), duty_cycles.begin(), duty_cycles.end());

    // Publish message
    dof_pud_.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller");
    ArmController arm_controller;

    ros::spin();
}
