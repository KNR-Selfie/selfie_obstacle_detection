/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

#include <gmock/gmock.h>

using sensor_msgs::LaserScanPtr;
using selfie_obstacle_detection::ObstacleObservations;
using selfie_obstacle_detection::IObstacleObservationsExtractor;
using selfie_obstacle_detection::CornerDetector;

class MockObstacleObservationsExtractor
	: public IObstacleObservationsExtractor
{
public:
	MOCK_METHOD1(extractObstacleObservations, ObstacleObservations(LaserScanPtr scan));
};

TEST(CornerDetectorTestSuite, basicTest)
{
	MockObstacleObservationsExtractor extractor;
	CornerDetector detector(&extractor);

	LaserScanPtr scan;

	EXPECT_CALL(extractor, extractObstacleObservations(scan));

	detector.detectCorners(scan);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
