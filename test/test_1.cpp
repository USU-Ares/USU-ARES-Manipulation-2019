// Compile with
// g++ -Wall -pthread test_1.cpp -lgtest_main -lgtest -pthread

#include <gtest/gtest.h>

#include "../include/nyx_pose.hpp"

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

TEST(PoseTest, PositiveShprecialDeltaPhi) {
    Pose a (1,1,1);
    Pose b (3.5,1,1);

    a.deltaSpherical(0.4,0,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}
TEST(PoseTest, NegativeShprecialDeltaPhi) {
    Pose a (1,1,1);
    Pose b (-7.2,1,1);

    a.deltaSpherical(-8.2,0,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

TEST(PoseTest, PositiveShprecialDeltaTheta) {
    Pose a (1,1,1);
    Pose b (1,2.2,1);

    a.deltaSpherical(0,1.2,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}
TEST(PoseTest, NegativeShprecialDeltaTheta) {
    Pose a (1,1,1);
    Pose b (1,-7.2,1);

    a.deltaSpherical(0,-8.2,0);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

TEST(PoseTest, PositiveShprecialDeltaRho) {
    Pose a (1,1,1);
    Pose b (1,1,1.2);

    a.deltaSpherical(0,0,0.2);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}
TEST(PoseTest, NegativeShprecialDeltaRho) {
    Pose a (1,1,1);
    Pose b (1,1,-4.3);

    a.deltaSpherical(0,0,-5.3);

    // Assert each value is equal
    ASSERT_NEAR(a.getX(), b.getX(), 1e-9);
    ASSERT_NEAR(a.getY(), b.getY(), 1e-9);
    ASSERT_NEAR(a.getZ(), b.getZ(), 1e-9);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
