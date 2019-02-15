/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

#include <gtest/gtest.h>

using sensor_msgs::LaserScanPtr;
using selfie_obstacle_detection::CornerDetector;

TEST(CornerDetectorTestSuite, basicTest)
{
	CornerDetector detector;

	LaserScanPtr scan;
	detector.detectCorners(scan);
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
