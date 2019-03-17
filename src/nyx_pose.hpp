#ifndef NYX_POSE_HPP
#define NYX_POSE_HPP

#include <math.h>
#include <stdio.h>

// Class to handle pose information for planning
class Pose {
    public:
        // Constuctors
        Pose();
        Pose(double x, double y, double z);

        // Accessors
        double getX() const { return x; }
        double getY() const { return y; }
        double getZ() const { return z; }

        // Modifiers
        void setX(double newX) { x = newX; }
        void setY(double newY) { y = newY; }
        void setZ(double newZ) { z = newZ; }

        // Distance between two poses
        double operator-(const Pose &rhs) const;

        void print() const;
    private:
        double x;
        double y;
        double z;
};

Pose::Pose() :
    Pose(-1,-1,-1)
{

}
Pose::Pose(double x1, double y1, double z1) :
    x(x1),
    y(y1),
    z(z1)
{

}

double Pose::operator-(const Pose &rhs) const {
    double distance = pow( x - rhs.getX(), 2) +
                      pow( y - rhs.getY(), 2) +
                      pow( z - rhs.getZ(), 2);
    distance = sqrt(distance);
    return distance;
}

void Pose::print() const {
    printf("X: %f  Y: %f  Z: %f\n",x,y,z);
}

#endif
