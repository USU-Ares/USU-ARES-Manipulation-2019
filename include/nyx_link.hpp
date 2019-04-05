#ifndef NYX_LINK_HPP
#define NYX_LINK_HPP

#include <stdio.h>
#include <vector>

// Helper function to convert degrees to radians
double radians(const double degrees) {
    return degrees * 3.14159265 / 180.0;
}

class Link {
    public:
        // Constructors
        Link();
        Link(double length, int axis, int min_theta, int max_theta, int min_duty, int max_duty);

        // Destructor
        ~Link();

        // Accessors
        double getLength();
        int getAngle() const;
        int getAxisOfRotation();

        // Return duty cycle from given angle
        int getDuty(int theta);

        // Return angle of given duty cycle
        int getAngle(int duty);

        // Validity checks
        bool isValidAngle(int theta);
        bool isValidDuty (int duty );
        bool isValid();

        // Set current angle for planning help
        void setCurrentAngle(int theta);
        void toMiddle();

        void print() const;

        double getX() const;
        double getY() const;
        double getZ() const;
    private:
        // Link information
        int axis;      // Direction of rotation. 1 = yaw, 2 = pitch
        double length; // Measured in units of length
        int min_theta; // Measured in degrees
        int max_theta; // Measured in degrees
        int min_duty;  // Minimum duty cycle for PWM
        int max_duty;  // Maximum duty cycle for PWM

        // State variables
        int current_theta; // Measured in degrees. 0 is in line with previous link
        int destination_theta; // State to plan to

        // Helper functions for conversion
        double anglePercentage(int angle);
        double dutyPercentage (int duty );
};

Link::Link() :
    Link(-1,-1,-1,-1,-1,0)
{
    // Call other constructor
}
Link::Link(double length, int axis, int min_theta, int max_theta, int min_duty, int max_duty) :
    axis      (axis),
    length    (length),
    min_theta (min_theta),
    max_theta (max_theta),
    min_duty  (min_duty),
    max_duty  (max_duty)
{
    // Set current_theta to be middle of range
    current_theta = (max_theta - min_theta)/2 + min_theta;
}
Link::~Link() {

}

// Accessors
double Link::getLength() {
    return length;
}
int Link::getAngle() const {
    return current_theta;
}
int Link::getAxisOfRotation() {
    return axis;
}

// Return duty cycle from given angle
int Link::getDuty(int theta) {
    // Get percentage of angle range
    double duty = anglePercentage(theta);
    // Multiply by duty range
    duty *= (max_duty-min_duty);
    // Add min duty and return
    return duty + min_duty;
}
// Return angle cycle from given duty cycle
int Link::getAngle(int duty) {
    // Get percentage of duty cycle range
    double theta = dutyPercentage(duty);
    // Multiply by theta range
    theta *= (max_theta-min_theta);
    // Add min theta and return
    return theta + min_theta;
}

// Set current angle of link
void Link::setCurrentAngle(int theta) {
    current_theta = theta;
}
// Set angle to be in the middle of range
void Link::toMiddle() {
    setCurrentAngle( (max_theta + min_theta) / 2);
}

// Return percentage of angle within range
double Link::anglePercentage(int theta) {
    if (!isValidAngle(theta)) {
        return 0;
    }
    return (double)(theta-min_theta) / (max_theta-min_theta);
}
// Return percentage of duty cycle within range
double Link::dutyPercentage(int duty) {
    if (!isValidDuty(duty)) {
        return 0;
    }
    return (double)(duty-min_duty) / (max_duty-min_duty);
}

bool Link::isValidAngle(int angle) {
    return angle >= min_theta && angle <= max_theta;
}
bool Link::isValidDuty(int duty) {
    return duty >= min_duty && duty <= max_duty;
}
bool Link::isValid() {
    return isValidAngle(current_theta);
}

void Link::print() const {
    printf("Theta: %d  Min/Max: %d/%d\n",current_theta, min_theta, max_theta);
}

double Link::getX() const {
    if (axis == 1) {
        // In XY plane
        return length * cos(current_theta);
    } else {
        // In XZ/YZ plane
        return length * sin(current_theta); // Will be 0
    }
}
double Link::getY() const {
    if (axis == 1) {
        // In XY plane
        return length * sin(current_theta);
    } else {
        // In XZ/YZ plane
        return length * cos(current_theta);
    }
}
double Link::getZ() const {
    if (axis == 1) {
        // In XY plane
        return 0;
    } else {
        // In XZ/YZ plane
        return length * sin(current_theta);
    }
}
#endif
