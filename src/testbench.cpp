#include <iostream>

#include "../include/nyx_solver.hpp"

int main() {
#if 0
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
#endif
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

    /*chain[0].setCurrentAngle(85);
    chain[2].setCurrentAngle(-50);
    chain[3].setCurrentAngle(-90);
    chain[4].setCurrentAngle(0);
    */
    
    Pose dest (35.611878, 3.115636, -1.454086);

    Solver solver (chain);

    std::cout << "Angles: " << std::endl;
    solver.getState().print();

    std::cout << "Solved state:" << std::endl;
    solver.updateTargetPose(dest).print();

}
