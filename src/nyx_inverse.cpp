#include <iostream>
#include <math.h>

/**
 * Class to keep track of link information
 */
class Link {

};

class InverseKinematics {
    public:
        // Default constructor
        InverseKinematics();

        // Add links
        void addLink(double length);

    private:
        int linkCount;
        double linkLength;
        double minAngle; // Measured in degrees
        double maxAngle; // Measured in degrees
};

// Add link of @length to end of chain
void InverseKinematics::addLink(double length) {

}
