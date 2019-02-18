/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

#include <gmock/gmock.h>
#include <algorithm>

using sensor_msgs::LaserScan;
using sensor_msgs::LaserScanPtr;

using std_msgs::Header;

using selfie_obstacle_detection::Corner;
using selfie_obstacle_detection::CornerPtr;
using selfie_obstacle_detection::CornerArrayPtr;
using selfie_obstacle_detection::Point;
using selfie_obstacle_detection::PointPtr;
using selfie_obstacle_detection::Line;
using selfie_obstacle_detection::LinePtr;
using selfie_obstacle_detection::ObstacleObservation;
using selfie_obstacle_detection::ObstacleObservationPtr;
using selfie_obstacle_detection::ObstacleObservations;
using selfie_obstacle_detection::IObstacleObservationsExtractor;
using selfie_obstacle_detection::ILineHelper;
using selfie_obstacle_detection::ICornerGenerator;
using selfie_obstacle_detection::CornerDetector;

using std::vector;
using std::find;
using std::prev;
using std::next;

using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::Lt;
using ::testing::Le;
using ::testing::Gt;
using ::testing::Ge;

#define CONTAINS(v, el) (find(v.begin(), v.end(), el) != v.end())

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

	MOCK_METHOD2(arePerpendicular, bool(LinePtr l1, LinePtr l2));
};

class MockCornerGenerator : public ICornerGenerator
{
public:
	MOCK_METHOD4(generateCorners, void(PointPtr firstPoint,
	                                   PointPtr secondPoint,
	                                   CornerPtr& firstCorner,
	                                   CornerPtr& secondCorner));
};

namespace std_msgs
{

bool operator==(const Header& h1, const Header& h2)
{
	return h1.frame_id == h2.frame_id;
}

} // namespace std_msgs

void fillHeader(Header& header)
{
	header.frame_id = "test";
}

namespace selfie_obstacle_detection
{

bool operator==(const Corner& c1, const Corner& c2)
{
	return c1.pose.position.x == c2.pose.position.x
	    && c1.pose.position.y == c2.pose.position.y
	    && c1.pose.position.z == c2.pose.position.z

	    && c1.pose.orientation.x == c2.pose.orientation.x
	    && c1.pose.orientation.y == c2.pose.orientation.y
	    && c1.pose.orientation.z == c2.pose.orientation.z
	    && c1.pose.orientation.w == c2.pose.orientation.w;
}

} // namespace selfie_obstacle_detection

TEST(CornerDetectorTestSuite, singleEdgeObservation)
{
	MockObstacleObservationsExtractor extractor;
	MockLineHelper helper;
	MockCornerGenerator generator;

	CornerDetector detector(&extractor, &helper, &generator);

	LaserScanPtr scan(new LaserScan());
	fillHeader(scan->header);

	ObstacleObservationPtr edgeObservation(new ObstacleObservation());
	for (int i = 0; i < 15; i++) edgeObservation->emplace_back(new Point(0, 0));

	ObstacleObservations observations = { edgeObservation };

	EXPECT_CALL(extractor, extractObstacleObservations(scan))
		.WillOnce(Return(observations));

	LinePtr edgeLine = LinePtr(new Line());

	EXPECT_CALL(helper, fitLineToSegment(_, _, _))
		.WillRepeatedly(DoAll(SetArgReferee<2>(edgeLine), Return(true)));

	PointPtr p1 = PointPtr(new Point(0, 0));
	PointPtr p2 = PointPtr(new Point(0, 0));

	EXPECT_CALL(helper, projectPointOntoLine(*edgeObservation->begin(), edgeLine))
		.WillOnce(Return(p1));

	EXPECT_CALL(helper, projectPointOntoLine(*prev(edgeObservation->end()), edgeLine))
		.WillOnce(Return(p2));

	CornerPtr c1 = CornerPtr(new Corner());
	c1->pose.position.x = 0.5;
	c1->pose.position.y = 2.3;

	CornerPtr c2 = CornerPtr(new Corner());
	c1->pose.position.x = 1.5;
	c1->pose.position.y = 0.2;

	EXPECT_CALL(generator, generateCorners(p1, p2, _, _))
		.WillOnce(DoAll(SetArgReferee<2>(c1), SetArgReferee<3>(c2)));

	CornerArrayPtr corners = detector.detectCorners(scan);

	EXPECT_EQ(corners->header, scan->header);

	EXPECT_EQ(corners->data.size(), 2);
	EXPECT_TRUE(CONTAINS(corners->data, *c1));
	EXPECT_TRUE(CONTAINS(corners->data, *c2));
}

TEST(CornerDetectorTestSuite, singleCornerObservation)
{
	MockObstacleObservationsExtractor extractor;
	MockLineHelper helper;
	MockCornerGenerator generator;

	CornerDetector detector(&extractor, &helper, &generator);

	LaserScanPtr scan(new LaserScan());

	ObstacleObservationPtr cornerObservation(new ObstacleObservation());
	for (int i = 0; i < 15; i++) cornerObservation->emplace_back(new Point(0, 0));
	ObstacleObservation::iterator cornerPointLocation = next(cornerObservation->begin(), 9);

	ObstacleObservations observations = { cornerObservation };

	EXPECT_CALL(extractor, extractObstacleObservations(_))
		.WillOnce(Return(observations));

	LinePtr l1 = LinePtr(new Line());
	LinePtr l2 = LinePtr(new Line());

	EXPECT_CALL(helper, fitLineToSegment(_, _, _))
		.WillRepeatedly(Return(false));

	EXPECT_CALL(helper, fitLineToSegment(Lt(cornerPointLocation), Le(cornerPointLocation), _))
		.WillRepeatedly(Return(true));

	EXPECT_CALL(helper, fitLineToSegment(Ge(cornerPointLocation), Gt(cornerPointLocation), _))
		.WillRepeatedly(Return(true));

	EXPECT_CALL(helper, fitLineToSegment(cornerObservation->begin(), prev(cornerPointLocation), _))
		.WillRepeatedly(DoAll(SetArgReferee<2>(l1), Return(true)));

	EXPECT_CALL(helper, fitLineToSegment(cornerPointLocation, cornerObservation->end(), _))
		.WillRepeatedly(DoAll(SetArgReferee<2>(l2), Return(true)));

	EXPECT_CALL(helper, arePerpendicular(l1, l2));

	detector.detectCorners(scan);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
