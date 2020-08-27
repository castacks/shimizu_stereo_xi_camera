
#include <iostream>

#include <gtest/gtest.h>

#include "ros/ros.h"

#include "AEAG/CentralMeanBrightness.hpp"

TEST(AEAGTestSuite, Test_CentralMeanBrightness_WriteInices) {
    std::cout << "Test_CentralMeanBrightness_WriteInices \n";

    // Create a CentralMeanBrightness AEAG object.
    sxc::CentralMeanBrightness cmb(4112, 3008, 0.5, 0.5, 100, 100, 
        1.0, 1.0, 1.0);

    // Write the indices.
    cmb.write_indices_as_image("/tmp/CentralMeanBrightness_SampledIndices.png");
}

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}