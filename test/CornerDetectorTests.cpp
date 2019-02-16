/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

#include <gmock/gmock.h>

using sensor_msgs::LaserScanPtr;

using selfie_obstacle_detection::PointXY;
using selfie_obstacle_detection::Line;
using selfie_obstacle_detection::ObstacleObservation;
using selfie_obstacle_detection::ObstacleObservations;
using selfie_obstacle_detection::IObstacleObservationsExtractor;
using selfie_obstacle_detection::ILineHelper;
using selfie_obstacle_detection::CornerDetector;

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;

class MockObstacleObservationsExtractor
	: public IObstacleObservationsExtractor
{
public:
	MOCK_METHOD1(extractObstacleObservations, ObstacleObservations(LaserScanPtr scan));
};

class MockLineHelper : public ILineHelper
{
public:
	MOCK_METHOD3(fitLineToSegment, bool(ObstacleObservation::iterator start,
	                                    ObstacleObservation::iterator end,
	                                    Line& line));
};

TEST(CornerDetectorTestSuite, singleEdgeObservation)
{
	MockObstacleObservationsExtractor extractor;
	MockLineHelper helper;
	CornerDetector detector(&extractor, &helper);

	LaserScanPtr scan;

	ObstacleObservation edgeObservation;
	for (int i = 0; i < 15; i++) edgeObservation.emplace_back(0, 0);

	ObstacleObservations observations = { edgeObservation };

	Line edgeLine;

	EXPECT_CALL(extractor, extractObstacleObservations(scan))
		.WillOnce(Return(observations));

	EXPECT_CALL(helper, fitLineToSegment(_, _, _))
		.WillRepeatedly(DoAll(SetArgReferee<2>(edgeLine), Return(true)));

	detector.detectCorners(scan);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
