#ifndef NYX_POSE_HPP
#define NYX_POSE_HPP

#include <math.h>
#include <stdio.h>

void cartesianToSpherical(double &x, double &y, double &z, double &rho, double &theta, double &phi);
void sphericalToCartesian(double &x, double &y, double &z, double &rho, double &theta, double &phi);

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

        // Deltas
        void deltaX(double d_x) { x += d_x; }
        void deltaY(double d_y) { y += d_y; }
        void deltaZ(double d_z) { z += d_z; }

        void deltaCartesian(double d_x, double d_y, double d_z);
        void deltaSpherical(double d_rho, double d_theta, double d_phi);

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

void Pose::deltaCartesian(double d_x, double d_y, double d_z) {
    deltaX(d_x);
    deltaY(d_y);
    deltaZ(d_z);
}
void Pose::deltaSpherical(double d_rho, double d_theta, double d_phi) {
    // Get spherical coordinates from cartesian points
    double rho, theta, phi;
    cartesianToSpherical(x,y,z, rho, theta, phi);

    // Adjust values by deltas
    phi += d_phi;
    theta += d_theta;
    rho += d_rho;

    // Convert back to cartesian
    sphericalToCartesian(x,y,z, rho, theta, phi);
}

void cartesianToSpherical(double &x, double &y, double &z, double &rho, double &theta, double &phi) {
    phi = atan( z / sqrt(x*x + y*y) );
    theta = atan( y / x );
    rho = sqrt( x*x + y*y + z*z);
}
void sphericalToCartesian(double &x, double &y, double &z, double &rho, double &theta, double &phi) {
    x = rho * cos(phi) * cos(theta);
    y = rho * cos(phi) * sin(theta);
    z = rho * sin(phi);
}

#endif
