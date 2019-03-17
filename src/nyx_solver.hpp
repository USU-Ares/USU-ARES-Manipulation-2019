#ifndef NYX_SOLVER_HPP
#define NYX_SOLVER_HPP

#include <vector>
#include <queue>
#include <unordered_map>

#include "nyx_link.hpp"
#include "nyx_pose.hpp"
#include "nyx_state.hpp"

class Solver {
    public:
        // Constructors
        Solver(std::vector<Link> chain);
        Solver(std::vector<Link> chain, Pose start);

        // Destructor
        ~Solver();

        void updateTargetPose(Pose newTarget); // Placeholder for testing, replace with joyCallback

        bool plan(); // Solve IK from current pose to target

        void applyMove(); // Use previous plan and update poses
    private:
        std::vector<Link> chain;

        // Pose information
        Pose currentPose;
        Pose targetPose;
};

Solver::Solver(std::vector<Link> chain) :
    Solver(chain, Pose())
{
}
Solver::Solver(std::vector<Link> chain, Pose start) :
    chain (chain),
    currentPose (start),
    targetPose (Pose())
{
}
Solver::~Solver() {
    /*if (p_currentPose != nullptr) {
        delete p_currentPose;
    }
    if (p_targetPose != nullptr) {
        delete p_targetPose;
    }
    p_currentPose = nullptr;
    p_targetPose  = nullptr;
    */
}

void Solver::updateTargetPose(Pose newTarget) {
    targetPose = newTarget;
    if (plan()) {
        std::cout << "Fould solution\n";
    } else {
        std::cout << "No solution found\n";
    }
}

bool Solver::plan() {
    // Min heap to keep track of states
    std::priority_queue<State, std::vector<State>, std::greater<State>> stateHeap;

    // Initial state
    State current = State(chain, currentPose, targetPose);
    stateHeap.push(current);

    // Hash to keep track of what has already been seen
    std::unordered_map<std::string, bool> mapper;

    // Counter for debug
    uint64_t count = 0;

    // Loop until we run out of things to search, or hit maximum search depth
    while (!stateHeap.empty() && stateHeap.size() < 100000) {
    //while (!stateHeap.empty()) {
        //std::cout << "Size of heap: " << stateHeap.size() << "\n";
        //std::cout << "Size of heap: " << stateHeap.size() << "\tMap Load: " << mapper.load_factor() << "\n";
        count++;
        
        current = stateHeap.top();
        stateHeap.pop();

        // Check if valid state
        if (current.forwardKinematics() - targetPose < 0.0001) {
            std::cout << "Found solution!\n";
            std::cout << "Iterations to find: " << count << "\n";
            current.print();
            return true;
        }

        //current.print();
        //std::cout << "\n";
        // Push next states onto heap
        std::vector<State> nextStates = current.getNext();
        for (int i=0; i<nextStates.size(); i++) {
            std::string index = nextStates[i].hash();
            if (mapper.count(index) == 0) {
                stateHeap.push(nextStates[i]);
                mapper[index] = true;
            }
        }
    }
    return false;
}

void Solver::applyMove() {
    currentPose = targetPose;
}

#endif
