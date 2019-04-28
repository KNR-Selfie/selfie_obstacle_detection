/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/ObstacleObservationsExtractor.h"

#include <gmock/gmock.h>

using sensor_msgs::LaserScan;
using sensor_msgs::LaserScanPtr;

using selfie_obstacle_detection::Point;
using selfie_obstacle_detection::PointPtr;
using selfie_obstacle_detection::ObstacleObservationPtr;
using selfie_obstacle_detection::ObstacleObservations;
using selfie_obstacle_detection::IMeasurementValidator;
using selfie_obstacle_detection::ICoordinatesTransformer;
using selfie_obstacle_detection::ObstacleObservationsExtractor;

using ::testing::_;
using ::testing::Return;

class MockMeasurementValidator
  : public IMeasurementValidator
{
public:
  MOCK_METHOD1(isValid, bool(float measurement));
};

class MockCoordinatesTransformer
  : public ICoordinatesTransformer
{
public:
  MOCK_METHOD3(transformCoordinates, PointPtr(LaserScanPtr scan, int index, float value));
};

TEST(ObstacleObservationsExtractorTestSuite, basicTest)
{
  MockMeasurementValidator validator;
  MockCoordinatesTransformer transformer;

  ObstacleObservationsExtractor extractor(&validator, &transformer);

  LaserScanPtr scan(new LaserScan());
  scan->ranges.push_back(0.0);
  scan->ranges.push_back(3.0);
  scan->ranges.push_back(5.0);
  scan->ranges.push_back(4.0);
  scan->ranges.push_back(0.0);
  scan->ranges.push_back(1.0);
  scan->ranges.push_back(2.0);

  EXPECT_CALL(validator, isValid(_))
  .WillRepeatedly(Return(true));

  EXPECT_CALL(validator, isValid(0.0))
  .WillRepeatedly(Return(false));

  PointPtr p11(new Point(0, 0));
  EXPECT_CALL(transformer, transformCoordinates(scan, 1, 3.0))
  .WillOnce(Return(p11));

  PointPtr p12(new Point(0, 0));
  EXPECT_CALL(transformer, transformCoordinates(scan, 2, 5.0))
  .WillOnce(Return(p12));

  PointPtr p13(new Point(0, 0));
  EXPECT_CALL(transformer, transformCoordinates(scan, 3, 4.0))
  .WillOnce(Return(p13));

  PointPtr p21(new Point(0, 0));
  EXPECT_CALL(transformer, transformCoordinates(scan, 5, 1.0))
  .WillOnce(Return(p21));

  PointPtr p22(new Point(0, 0));
  EXPECT_CALL(transformer, transformCoordinates(scan, 6, 2.0))
  .WillOnce(Return(p22));

  ObstacleObservations observations = extractor.extractObstacleObservations(scan);
  ASSERT_EQ(observations.size(), 2);

  ObstacleObservationPtr obs1 = observations[0];
  ASSERT_EQ(obs1->size(), 3);
  EXPECT_EQ((*obs1)[0], p11);
  EXPECT_EQ((*obs1)[1], p12);
  EXPECT_EQ((*obs1)[2], p13);

  ObstacleObservationPtr obs2 = observations[1];
  ASSERT_EQ(obs2->size(), 2);
  EXPECT_EQ((*obs2)[0], p21);
  EXPECT_EQ((*obs2)[1], p22);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
