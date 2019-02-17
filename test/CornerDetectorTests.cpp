/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

#include <gmock/gmock.h>

using sensor_msgs::LaserScanPtr;

using selfie_obstacle_detection::Corner;
using selfie_obstacle_detection::CornerPtr;
using selfie_obstacle_detection::Point;
using selfie_obstacle_detection::PointPtr;
using selfie_obstacle_detection::Line;
using selfie_obstacle_detection::LinePtr;
using selfie_obstacle_detection::ObstacleObservation;
using selfie_obstacle_detection::ObstacleObservations;
using selfie_obstacle_detection::IObstacleObservationsExtractor;
using selfie_obstacle_detection::ILineHelper;
using selfie_obstacle_detection::ICornerGenerator;
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
	                                    LinePtr& line));

	MOCK_METHOD2(projectPointOntoLine, PointPtr(PointPtr point, LinePtr line));
};

class MockCornerGenerator : public ICornerGenerator
{
public:
	MOCK_METHOD4(generateCorners, void(PointPtr firstPoint,
	                                   PointPtr secondPoint,
	                                   CornerPtr& firstCorner,
	                                   CornerPtr& secondCorner));
};

TEST(CornerDetectorTestSuite, singleEdgeObservation)
{
	MockObstacleObservationsExtractor extractor;
	MockLineHelper helper;
	MockCornerGenerator generator;

	CornerDetector detector(&extractor, &helper, &generator);

	LaserScanPtr scan;

	ObstacleObservation edgeObservation;
	for (int i = 0; i < 15; i++) edgeObservation.emplace_back(new Point(0, 0));

	ObstacleObservations observations = { edgeObservation };

	EXPECT_CALL(extractor, extractObstacleObservations(scan))
		.WillOnce(Return(observations));

	LinePtr edgeLine = LinePtr(new Line());

	EXPECT_CALL(helper, fitLineToSegment(_, _, _))
		.WillRepeatedly(DoAll(SetArgReferee<2>(edgeLine), Return(true)));

	PointPtr p1 = PointPtr(new Point(0, 0));
	PointPtr p2 = PointPtr(new Point(0, 0));

	EXPECT_CALL(helper, projectPointOntoLine(*edgeObservation.begin(), edgeLine))
		.WillOnce(Return(p1));

	EXPECT_CALL(helper, projectPointOntoLine(*std::prev(edgeObservation.end()), edgeLine))
		.WillOnce(Return(p2));

	CornerPtr c1 = CornerPtr(new Corner());
	CornerPtr c2 = CornerPtr(new Corner());

	EXPECT_CALL(generator, generateCorners(p1, p2, _, _))
		.WillOnce(DoAll(SetArgReferee<2>(c1), SetArgReferee<3>(c2)));

	detector.detectCorners(scan);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
