#include <iostream>

#include "nyx_solver.hpp"

int main(int argc, char** argv) {
#if 0
    Pose firstPose(2,0,0);
    Pose secondPose(1,1,0);
    //Pose secondPose(-0.330083,1.972496,0);
    //Pose secondPose(0.8,0,0);
    //Pose secondPose(0,2,0);

    std::vector<Link> chain = std::vector<Link>();

    chain.push_back(Link(1, 1, -15,15, 0,180));
    chain.push_back(Link(1, 1, -15,15, 0,180));

    std::cout << "Creating state" << std::endl;

    State s = State(chain, firstPose, secondPose);

    //std::cout << "State created" << std::endl;
    //std::cout << "Current state" << std::endl;
    //std::cout << s.print();
#else
    Pose firstPose(40,0,0);
    //Pose secondPose(3,1,0);
    //Pose secondPose(0.964322,3.807231,0);
    //Pose secondPose(20,34.6, 0);
    //Pose secondPose(20.6,34.26, 0);
    //Pose secondPose(50, 0,0);
    //Pose secondPose(-30, 10, 0);
    //Pose secondPose(10,30,0);
    //Pose secondPose(0,30,10);
    //Pose secondPose(30,0,10);
    Pose secondPose(21.21, 21.21, 10);

    std::vector<Link> chain = std::vector<Link>();

    chain.push_back(Link(10, 1, -90,90, 0,180));
    chain.push_back(Link(10, 2, -90,90, 0,180));
    chain.push_back(Link(10, 2, -90,90, 0,180));
    chain.push_back(Link(10, 2, -90,90, 0,180));
#endif

    Solver nyx_solver = Solver(chain, firstPose);

    nyx_solver.updateTargetPose(secondPose);

    /*std::cout << "\n\nAttempting to get next state\n";
    std::vector<State> nextStates = s.getNext();

    for (int i=0; i<nextStates.size(); i++) {
        std::cout << "\nI: " << i << "\n";
        nextStates[i].print();
    }

    std::cout << "second level states, based off of index 0\n";
    for (int j=0; j<10; j++) {
        nextStates = nextStates[0].getNext();
        for (int i=0; i<nextStates.size(); i++) {
            std::cout << "\nI: " << i << "\n";
            nextStates[i].print();
        }
    }
    */
}
