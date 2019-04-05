// Compile with
// g++ -Wall -pthread test_1.cpp -lgtest_main -lgtest -pthread -larmadillo

#include <gtest/gtest.h>

#include "../include/nyx_pose.hpp"
#include "../include/nyx_link.hpp"
#include "../include/nyx_state.hpp"
#include "../include/nyx_solver.hpp"

// *******************************************
// BEGIN POSE TESTS
// *******************************************

TEST(PoseTest, ZeroDeltaCartesian) {
    Pose a (1,1,1);
    Pose b (1,1,1);

    a.deltaCartesian(0,0,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

TEST(PoseTest, ZeroDeltaSpherical) {
    Pose a (1,1,1);
    Pose b (1,1,1);

    a.deltaSpherical(0,0,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

// CARTESIAN TESTS

TEST(PoseTest, PositiveCartesianDeltaX) {
    Pose a (1,1,1);
    Pose b (3.5,1,1);

    a.deltaCartesian(2.5,0,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}
TEST(PoseTest, NegativeCartesianDeltaX) {
    Pose a (1,1,1);
    Pose b (-7.2,1,1);

    a.deltaCartesian(-8.2,0,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

TEST(PoseTest, PositiveCartesianDeltaY) {
    Pose a (1,1,1);
    Pose b (1,2.2,1);

    a.deltaCartesian(0,1.2,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}
TEST(PoseTest, NegativeCartesianDeltaY) {
    Pose a (1,1,1);
    Pose b (1,-7.2,1);

    a.deltaCartesian(0,-8.2,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

TEST(PoseTest, PositiveCartesianDeltaZ) {
    Pose a (1,1,1);
    Pose b (1,1,1.2);

    a.deltaCartesian(0,0,0.2);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}
TEST(PoseTest, NegativeCartesianDeltaZ) {
    Pose a (1,1,1);
    Pose b (1,1,-4.3);

    a.deltaCartesian(0,0,-5.3);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

// SPHERICAL TESTS

TEST(PoseTest, PositiveSphrecialDeltaTheta) {
    Pose a (1,1,1);
    Pose b (0.781387, 1.178736, 1);

    a.deltaSpherical(0,0.2,0);

    // Assert each value is equal
    EXPECT_NEAR(a.getX(), b.getX(), 1e-4);
    EXPECT_NEAR(a.getY(), b.getY(), 1e-4);
    EXPECT_NEAR(a.getZ(), b.getZ(), 1e-4);
}
TEST(PoseTest, NegativeSphrecialDeltaTheta) {
    Pose a (1,1,1);
    Pose b (1.178736, 0.781387, 1);

    a.deltaSpherical(0,-0.2,0);

    // Assert each value is equal
    EXPECT_NEAR(a.getX(), b.getX(), 1e-4);
    EXPECT_NEAR(a.getY(), b.getY(), 1e-4);
    EXPECT_NEAR(a.getZ(), b.getZ(), 1e-4);
}

TEST(PoseTest, PositiveSphrecialDeltaPhi) {
    Pose a (1,1,1);
    Pose b (0.8395862, 0.8395862, 1.261027);

    a.deltaSpherical(0,0,0.2);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-4);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-4);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-4);
}
TEST(PoseTest, NegativeSphrecialDeltaPhi) {
    Pose a (1,1,1);
    Pose b (1.164301, 1.164301, 0.537408);

    a.deltaSpherical(0,0,-0.3);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-4);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-4);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-4);
}

TEST(PoseTest, SphericalDeltaThetaDeltaPhi) {
    Pose a (1,1,1);
    Pose b (0.4242750, 1.446024, 0.8538184);

    a.deltaSpherical(0, 0.5, -0.1);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-4);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-4);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-4);
}

// *******************************************
// BEGIN LINK TESTS
// *******************************************

TEST(LinkTest, DutyFromAngle) {
    Link a (1, 1, -37, 92, 16, 170);

    // Positive angle
    int positiveAngle = 42;
    int positiveExpectedDuty = 110;
    // Zero angle
    int zeroAngle = 0;
    int zeroExpectedDuty = 60;
    // Negative angle
    int negativeAngle = -20;
    int negativeExpectedDuty = 36;
    // Min angle
    int minAngle = -37;
    int minExpectedDuty = 16;
    // Max angle
    int maxAngle = 92;
    int maxExpectedDuty = 170;

    ASSERT_EQ( a.getDuty( positiveAngle ), positiveExpectedDuty );
    ASSERT_EQ( a.getDuty( zeroAngle     ), zeroExpectedDuty     );
    ASSERT_EQ( a.getDuty( negativeAngle ), negativeExpectedDuty );
    ASSERT_EQ( a.getDuty( minAngle      ), minExpectedDuty      );
    ASSERT_EQ( a.getDuty( maxAngle      ), maxExpectedDuty      );
}
TEST(LinkTest, AngleFromDuty) {
    Link a (1, 1, -37, 92, 16, 170);

    // Positive angle
    int positiveAngle = 41;
    int positiveExpectedDuty = 110;
    // Zero angle
    int zeroAngle = 0;
    int zeroExpectedDuty = 60;
    // Negative angle
    int negativeAngle = -20;
    int negativeExpectedDuty = 36;
    // Min angle
    int minAngle = -37;
    int minExpectedDuty = 16;
    // Max angle
    int maxAngle = 92;
    int maxExpectedDuty = 170;

    ASSERT_EQ( a.getAngle( positiveExpectedDuty ), positiveAngle );
    ASSERT_EQ( a.getAngle( zeroExpectedDuty     ), zeroAngle     );
    ASSERT_EQ( a.getAngle( negativeExpectedDuty ), negativeAngle );
    ASSERT_EQ( a.getAngle( minExpectedDuty      ), minAngle      );
    ASSERT_EQ( a.getAngle( maxExpectedDuty      ), maxAngle      );
}

// *******************************************
// BEGIN STATE TESTS
// *******************************************
TEST(StateTest, ForwardKinematicsTest1) {
    std::vector<Link> chain;

    // Create chain
    chain.push_back( Link(1, 1, -90, 90, 0, 180) );
    chain.push_back( Link(1, 1,   0, 90, 0, 180) );
    chain.push_back( Link(1, 1, -45, 90, 0, 180) );

    // Center all links
    for (unsigned int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    State a(chain);

    Pose actualPose = a.forwardKinematics();
    Pose expectedPose (2.0897902, 1.6309863, 0);

    ASSERT_NEAR( actualPose.getX(), expectedPose.getX(), 1e-2);
    ASSERT_NEAR( actualPose.getY(), expectedPose.getY(), 1e-2);
    ASSERT_NEAR( actualPose.getZ(), expectedPose.getZ(), 1e-2);
}
TEST(StateTest, ForwardKinematicsTest2) {
    std::vector<Link> chain;

    // Create chain
    chain.push_back( Link(1, 2,   0, 90, 0, 180) );
    chain.push_back( Link(1, 1, -45, 90, 0, 180) );

    // Center all links
    for (unsigned int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    State a(chain);

    Pose actualPose = a.forwardKinematics();
    //Pose expectedPose (2.0897902, 1.6309863, 0);
    Pose expectedPose (1.33346, 0.3751969, 1.33346);

    EXPECT_NEAR( actualPose.getX(), expectedPose.getX(), 1e-2);
    EXPECT_NEAR( actualPose.getY(), expectedPose.getY(), 1e-2);
    EXPECT_NEAR( actualPose.getZ(), expectedPose.getZ(), 1e-2);
}
TEST(StateTest, ForwardKinematicsTest3) {
    std::vector<Link> chain;

    // Create chain
    chain.push_back( Link(1, 1, -90, 90, 0, 180) );
    chain.push_back( Link(1, 2,   0, 90, 0, 180) );
    chain.push_back( Link(1, 1, -45, 90, 0, 180) );

    // Center all links
    for (unsigned int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    State a(chain);

    Pose actualPose = a.forwardKinematics();
    //Pose expectedPose (2.360388, 0.2705981, 1.4142136);
    Pose expectedPose (1,1,1);

    ASSERT_NEAR( actualPose.getX(), expectedPose.getX(), 1e-2);
    ASSERT_NEAR( actualPose.getY(), expectedPose.getY(), 1e-2);
    ASSERT_NEAR( actualPose.getZ(), expectedPose.getZ(), 1e-2);
}
TEST(StateTest, ForwardKinematicsTest4) {
    std::vector<Link> chain;

    // Create chain
    chain.push_back( Link(1.2, 1, -90, 90, 0, 180) );
    chain.push_back( Link(1  , 2,   0, 90, 0, 180) );
    chain.push_back( Link(2.3, 1, -45, 90, 0, 180) );

    // Center all links
    for (unsigned int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    State a(chain);

    Pose actualPose = a.forwardKinematics();
    //Pose expectedPose (2.5603883, 0.62237552, );
    Pose expectedPose (1,1,1);

    ASSERT_NEAR( actualPose.getX(), expectedPose.getX(), 1e-2);
    ASSERT_NEAR( actualPose.getY(), expectedPose.getY(), 1e-2);
    ASSERT_NEAR( actualPose.getZ(), expectedPose.getZ(), 1e-2);
}
TEST(StateTest, OffsetForwardKinematicsTest1) {
    std::vector<Link> chain;

    // Create chain
    chain.push_back( Link(1, 1, -90,  90, 0, 180) );
    chain.push_back( Link(1, 2,  90,  90, 0, 180) );
    chain.push_back( Link(0, 2, -90, -90, 0, 180) );
    chain.push_back( Link(1, 1,   0,  90, 0, 180) );
    chain.push_back( Link(1, 1, -45,  90, 0, 180) );

    // Center all links
    for (unsigned int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    State a(chain);

    Pose actualPose = a.forwardKinematics();
    Pose expectedPose (2.0897902, 1.6309863, 1);

    ASSERT_NEAR( actualPose.getX(), expectedPose.getX(), 1e-2);
    ASSERT_NEAR( actualPose.getY(), expectedPose.getY(), 1e-2);
    ASSERT_NEAR( actualPose.getZ(), expectedPose.getZ(), 1e-2);
}
TEST(StateTest, OffsetForwardKinematicsTest2) {
    std::vector<Link> chain;

    // Create chain
    chain.push_back( Link(1, 1, -90,  90, 0, 180) );
    chain.push_back( Link(1, 2,  90,  90, 0, 180) );
    chain.push_back( Link(1, 2, -90, -90, 0, 180) );
    chain.push_back( Link(1, 1,   0,  90, 0, 180) );
    chain.push_back( Link(1, 1, -45,  90, 0, 180) );

    // Center all links
    for (unsigned int i=0; i<chain.size(); i++) {
        chain[i].toMiddle();
    }

    State a(chain);

    Pose actualPose = a.forwardKinematics();
    //Pose expectedPose (3.0897902, 1.6309863, 1);
    Pose expectedPose (1,1,1);

    ASSERT_NEAR( actualPose.getX(), expectedPose.getX(), 1e-2);
    ASSERT_NEAR( actualPose.getY(), expectedPose.getY(), 1e-2);
    ASSERT_NEAR( actualPose.getZ(), expectedPose.getZ(), 1e-2);
}

// *******************************************
// BEGIN SOLVER TESTS
// *******************************************

TEST(SolverTest, PlanToSamePosition) {
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
