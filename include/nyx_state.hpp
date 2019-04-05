#ifndef NYX_STATE_HPP
#define NYX_STATE_HPP

#include <vector>
#include <armadillo>

#include "nyx_link.hpp"
#include "nyx_pose.hpp"

// Class to keep track of state of arm during solve
class State {
    public:
        // Constructor
        State(std::vector<Link> chain);
        State(std::vector<Link> chain, Pose currentPose, Pose destinationPose);
        State(const State &state);

        // Create children in tree, return vector of States
        std::vector<State> getNext();
        std::vector<State> getNext(int stepSize);

        // Operator overloading to compare state scores
        bool operator>(const State &rhs) const;
        bool operator==(const State &rhs);

        void print() const;
        double getScore() const;

        std::string hash() const;
        Pose forwardKinematics() const;
        bool withinTolerance(const State &rhs, int tolerance) const;

        void setChain(std::vector<Link> chain);

        std::vector<Link> getChain();
    protected:
        std::vector<Link> chain;

    private:
        State();

        Pose currentPose;
        Pose destinationPose;

        int stepCount;
};

State::State(std::vector<Link> chain) :
    State(chain, Pose(), Pose())
{
}

State::State(std::vector<Link> chain, Pose currentPose, Pose destinationPose) :
    chain (chain),
    currentPose (currentPose),
    destinationPose (destinationPose)
{
    stepCount = 0;
}
State::State(const State &state) {
    this->chain = state.chain;
    this->currentPose = state.currentPose;
    this->destinationPose = state.destinationPose;
    this->stepCount = state.stepCount + 1;
}

std::vector<State> State::getNext() {
    return getNext(1);
}
std::vector<State> State::getNext(int stepSize) {
    // Create all next states that can result from current configuration
    std::vector<State> out = std::vector<State>();

    for (unsigned int i=0; i<chain.size(); i++) {
        // Get link from chain, and increase and decrease angle by 1 degree
        Link decrement = chain[i];
        decrement.setCurrentAngle(decrement.getAngle() - stepSize);
        // Create states that will be put into vector
        State first  = State(*this);
        // Check if valid link
        if (decrement.isValid()) {
            // Update link angles in each state
            first.chain[i] = decrement;
            // Push states to vector
            out.push_back(first);
        }


        Link increment = chain[i];
        increment.setCurrentAngle(increment.getAngle() + stepSize);
        State second = State(*this);
        if (increment.isValid()) {
            second.chain[i] = increment;
            out.push_back(second);
        }
    }

    return out;
}

void State::print() const {
    std::cout << "State information\n";
    currentPose.print();
    currentPose.print();
    for (unsigned int i=0; i<chain.size(); i++) {
        //output += "  " + chain[i].print() + "\n";
        chain[i].print();
    }
    std::cout << "End pose: ";
    Pose temp = forwardKinematics();
    temp.print();
}
double State::getScore() const {
    // Constants to multiply by
    double pastCoefficient   = 0.3;
    double futureCoefficient = 0.7;

    // Get distance between end effector and destinationPose
    double dist = destinationPose - forwardKinematics();

    return pastCoefficient * stepCount + futureCoefficient * dist;
}

bool State::operator>(const State &rhs) const {
    return (*this).getScore() > rhs.getScore();
}
bool State::operator==(const State &rhs) {
    return (*this).getScore() == rhs.getScore();
}

// Hash of this state
std::string State::hash() const {
    std::string out = "";
    for (unsigned int i=0; i<chain.size(); i++) {
        Link temp = chain[i];
        out += temp.getAngle();
    }
    return out;
}

// Get position of end manipulator with @chain
Pose State::forwardKinematics() const {
    // Initialize lastValue to be the final vector V
    double x, y, z;
    double rho, theta, phi;
    double c_phi, c_theta,
           s_phi, s_theta;

    Link current = chain[chain.size()-1];
    // Get X/Y/Z of link
    x = current.getX();
    y = current.getY();
    z = current.getZ();
    // Get angles
    cartesianToSpherical(x,y,z, rho,theta,phi);
    c_phi = cos(phi);
    s_phi = sin(phi);
    c_theta = cos(theta);
    s_theta = sin(theta);

    /*
    arma::dmat lastValue { rho * c_phi * c_theta ,
                           rho * c_phi * s_theta ,
                           rho * s_phi           };
                           */
    //std::cout << "Creating first lastValue\n";
    arma::dmat lastValue (3,1);
    lastValue[0] = rho * c_phi * c_theta;
    lastValue[1] = rho * c_phi * s_theta;
    lastValue[2] = rho * s_phi;
    //std::cout << "Finish first lastValue\n";

    // Loop through the links backwards, and apply the coordinate transforms
    for (int i=chain.size()-2; i>=0; i--) {
        // Y
        // [ cos(phi), 0, sin(phi),
        //   0       , 1, 0       ,
        //  -sin(phi), 0, cos(phi) ]
        //
        //  Z
        //  [ cos(theta), -sin(theta), 0,
        //    sin(theta),  cos(theta), 0,
        //    0         , 0          , 1 ]

        //std::cout << "A + " <<i<< "\n";
        x = chain[i].getX();
        y = chain[i].getY();
        z = chain[i].getZ();
        //std::cout << "B\n";
        // Get angles
        cartesianToSpherical(x,y,z, rho,theta,phi);
        c_phi = cos(phi);
        s_phi = sin(phi);
        c_theta = cos(theta);
        s_theta = sin(theta);
        //std::cout << "C\n";

        // Coordinate transform matrix
        arma::dmat y { { c_phi, 0 , s_phi},
                       {  0 , 1 ,  0 },
                       {-s_phi, 0 ,  c_phi} };
        //std::cout << "D\n";
        arma::dmat z { { c_theta, -s_theta, 0 },
                       { s_theta,  c_theta, 0 },
                       {  0 ,   0 , 1 } };

        //std::cout << "E\n";
        arma::dmat v (3,1);
        v[0] = rho * c_phi * c_theta;
        v[1] = rho * c_phi * s_theta;
        v[2] = rho * s_phi;;
        //std::cout << "F\n";

        lastValue = v + y * z * lastValue;
        //std::cout << "G\n";
    }

    //std::cout << "H\n";
    Pose end = Pose(lastValue[0], lastValue[1], lastValue[2]);
    //std::cout << "I\n";
    return end;
    /*
    // Initial pose
    Pose end = Pose(0,0,0);

    // End effector orientation
    double yaw = 0.0;
    double pitch = 0.0;

    // Loop through links and adjust pose and orientation
    for (unsigned int i=0; i<chain.size(); i++) {
        Link temp = chain[i];
        // If yaw joint, apply yaw
        if (temp.getAxisOfRotation() == 1) {
            yaw += temp.getAngle();
        }
        // If pitch joint, apply pitch
        if (temp.getAxisOfRotation() == 2) {
            pitch += temp.getAngle();
        }

        // Adjust pose appropriately

        end.setX( end.getX() + temp.getLength()*cos(radians(pitch))*cos(radians(yaw)) );
        end.setY( end.getY() + temp.getLength()*cos(radians(pitch))*sin(radians(yaw)) );
        end.setZ( end.getZ() + temp.getLength()*sin(radians(pitch)) );
    }

    return end;
    */
}

// Check that state is within bounds to given state
bool State::withinTolerance(const State &rhs, int tolerance) const {
    for (unsigned int i=0; i<chain.size(); i++) {
        if ( abs(chain[i].getAngle() - rhs.chain[i].getAngle()) > tolerance ) {
            return false;
        }
    }
    return true;
}

// Set the chain, this may need to be removed after testing
void State::setChain(std::vector<Link> chain) {
    this->chain = chain;
}

// Get chain
std::vector<Link> State::getChain() {
    return chain;
}

#endif
