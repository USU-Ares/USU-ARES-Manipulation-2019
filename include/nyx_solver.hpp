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
        Solver();
        Solver(std::vector<Link> chain);

        // Destructor
        ~Solver();

        void addSavedPosition(State position);

        State goToSavedPosition(int index);

        State updateTargetPose(Pose newTarget); // Placeholder for testing, replace with joyCallback

        State plan(); // Solve IK from current pose to target

        void applyMove(); // Use previous plan and update poses

        Pose getTarget();
        std::vector<Link> getChain();
        void setChain(std::vector<Link> n_chain);
        State getState();
    private:
        std::vector<Link> chain;

        // Saved positions
        std::vector<State> savedPositions;

        // Pose information
        Pose currentPose;
        Pose targetPose;

        State currentState;

        // Trim tree function
        State trim(State initial, int stepSize);
};

Solver::Solver() :
    Solver(std::vector<Link>())
{
}
Solver::Solver(std::vector<Link> chain) :
    currentState (State(chain))
{
    this->chain = chain;
    // Create current Pose information
    this->currentPose = currentState.forwardKinematics();
    this->targetPose = currentState.forwardKinematics();
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
void Solver::addSavedPosition(State position) {
    savedPositions.push_back(position);
}
State Solver::goToSavedPosition(int index) {
    // Check a valid entry
    /*if (index < 0 || index >= savedPositions.size()) {
        return false;
    }*/
    return updateTargetPose( savedPositions[index].forwardKinematics() );
}

State Solver::updateTargetPose(Pose newTarget) {
    // Store old target in case the new one is unreachable
    Pose oldTarget = targetPose;
    
    // Get solution to target
    targetPose = newTarget;
    State solution = plan();

    // Check if distance is too far
    if (solution.forwardKinematics() - targetPose > 1) {
        targetPose = oldTarget;
        return plan();
    }
    // Otherwise, return solved solution
    return solution;

    /*(if (plan()) {
        std::cout << "Fould solution\n";
        //return true;
    } else {
        std::cout << "No solution found\n";
        std::cout << "End: " << targetPose.getX() << "\t" << targetPose.getY() << "\t" << targetPose.getZ() << "\n";
        //return false;
    }*/
}

State Solver::plan() {
    int stepSize = 5;

    // Min heap to keep track of states
    std::priority_queue<State, std::vector<State>, std::greater<State>> stateHeap;

    // Hash to keep track of what has already been seen
    std::unordered_map<std::string, bool> mapper;

    // Counter for debug
    uint64_t count = 0;

    // Initial state
    State current = State(chain, currentPose, targetPose);

    // Trim tree
    State initial = trim(current, stepSize);
    stateHeap.push( initial );

    // DEBUG
    State closest = current;
    double nearestSolution = 1000.0;

    // Loop until we run out of things to search, or hit maximum search depth
    while (!stateHeap.empty() && stateHeap.size() < 500000) {
    //while (!stateHeap.empty()) {
        //std::cout << "Size of heap: " << stateHeap.size() << "\n";
        //std::cout << "Size of heap: " << stateHeap.size() << "\tMap Load: " << mapper.load_factor() << "\n";
        count++;
        
        current = stateHeap.top();
        stateHeap.pop();

        // Check if solution state
        double dist = current.forwardKinematics() - targetPose;
        //if (dist < 0.1) {
        if (dist < 1) {
            std::cout << "Found solution!\n";
            std::cout << "Iterations to find: " << count << "\n";
            current.print();
            // Set state
            chain = current.getChain();
            currentState = current;
            std::cout << "\t\t\t" << chain[0].getDuty(chain[0].getCurrentAngle()) << "\n";
            applyMove();

            //return true;
            return current;
        }
        if (dist < nearestSolution) {
            closest = current;
            nearestSolution = dist;
        }

        //current.print();
        //std::cout << "\n";
        // Push next states onto heap
        std::vector<State> nextStates = current.getNext();
        for (int i=0; i<nextStates.size(); i++) {
            std::string index = nextStates[i].hash();
            if (initial.withinTolerance(nextStates[i], stepSize) && mapper.count(index) == 0) {
                stateHeap.push(nextStates[i]);
                mapper[index] = true;
            }
        }
    }

#if 1
    std::cout << "Nearest found\n";
    current.print();
    std::cout << "Dist: " << nearestSolution << "\n";
#endif
    return current;
}

void Solver::applyMove() {
    currentPose = targetPose;
}

State Solver::trim(State initial, int stepSize) {
    // Trim tree to be closer to search destination
    std::vector<State> nextStates;
    State best = initial;

    bool loop = true;

    while (loop) {
        loop = false;
        nextStates = best.getNext(stepSize);

        // Find the state that is the closest to the target pose
        double distance = best.forwardKinematics() - targetPose;

        for (int i=0; i<nextStates.size(); i++) {
            Pose current_end = nextStates[i].forwardKinematics();

            // Check that our current state is closer from target than initial state
            if ( distance > current_end - targetPose ) {
                best = nextStates[i];
                distance = current_end - targetPose;
                loop = true;
            }
        }
    }

    //std::cout << "Finished trimming\n";

    return best;
}

Pose Solver::getTarget() {
    return targetPose;
}
std::vector<Link> Solver::getChain() {
    return chain;
}
void Solver::setChain(std::vector<Link> n_chain) {
    chain = n_chain;
}
State Solver::getState() {
    return currentState;
}

#endif
