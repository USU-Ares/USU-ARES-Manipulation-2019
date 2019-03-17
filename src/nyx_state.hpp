#ifndef NYX_STATE_HPP
#define NYX_STATE_HPP

#include <vector>

#include "nyx_link.hpp"
#include "nyx_pose.hpp"

// Class to keep track of state of arm during solve
class State {
    public:
        // Constructor
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

    protected:
        std::vector<Link> chain;

    private:
        State();

        Pose currentPose;
        Pose destinationPose;

        int stepCount;
};

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

    for (int i=0; i<chain.size(); i++) {
        // Get link from chain, and increase and decrease angle by 1 degree
        Link decrement = chain[i];
        decrement.setCurrentAngle(decrement.getCurrentAngle() - stepSize);
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
        increment.setCurrentAngle(increment.getCurrentAngle() + stepSize);
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
    for (int i=0; i<chain.size(); i++) {
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
    for (int i=0; i<chain.size(); i++) {
        Link temp = chain[i];
        out += temp.getCurrentAngle();
    }
    return out;
}

// Get position of end manipulator with @chain
Pose State::forwardKinematics() const {
    // Initial pose
    Pose end = Pose(0,0,0);

    // End effector orientation
    double yaw = 0.0;
    double pitch = 0.0;

    // Loop through links and adjust pose and orientation
    for (int i=0; i<chain.size(); i++) {
        Link temp = chain[i];
        // If yaw joint, apply yaw
        if (temp.getAxisOfRotation() == 1) {
            yaw += temp.getCurrentAngle();
        }
        // If pitch joint, apply pitch
        if (temp.getAxisOfRotation() == 2) {
            pitch += temp.getCurrentAngle();
        }

        // Adjust pose appropriately

        end.setX( end.getX() + temp.getLength()*cos(radians(pitch))*sin(radians(yaw)) );
        end.setY( end.getY() + temp.getLength()*cos(radians(pitch))*cos(radians(yaw)) );
        end.setZ( end.getZ() + temp.getLength()*sin(radians(pitch)) );
    }

    return end;
}

// Check that state is within bounds to given state
bool State::withinTolerance(const State &rhs, int tolerance) const {
    for (int i=0; i<chain.size(); i++) {
        if ( abs(chain[i].getCurrentAngle() - rhs.chain[i].getCurrentAngle()) > tolerance ) {
            return false;
        }
    }
    return true;
}

#endif
