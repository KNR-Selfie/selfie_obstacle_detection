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
  scan->ranges.push_back(0.0);

  EXPECT_CALL(validator, isValid(_))
  .WillRepeatedly(Return(true));

  EXPECT_CALL(validator, isValid(0.0))
  .WillRepeatedly(Return(false));

  PointPtr point(new Point(0, 0));
  EXPECT_CALL(transformer, transformCoordinates(scan, 1, 3.0))
  .WillOnce(Return(point));

  EXPECT_CALL(transformer, transformCoordinates(scan, 2, 5.0))
  .WillOnce(Return(point));

  extractor.extractObstacleObservations(scan);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
