/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/ObstacleObservationsExtractor.h"

#include <gmock/gmock.h>

using sensor_msgs::LaserScan;
using sensor_msgs::LaserScanPtr;

using selfie_obstacle_detection::IMeasurementValidator;
using selfie_obstacle_detection::ICoordinatesTransformer;
using selfie_obstacle_detection::ObstacleObservationsExtractor;

using ::testing::_;

class MockMeasurementValidator
  : public IMeasurementValidator
{
public:
  MOCK_METHOD1(isValid, bool(float measurement));
};

class MockCoordinatesTransformer
  : public ICoordinatesTransformer
{ };

TEST(ObstacleObservationsExtractorTestSuite, basicTest)
{
  MockMeasurementValidator validator;
  MockCoordinatesTransformer transformer;

  ObstacleObservationsExtractor extractor(&validator, &transformer);

  LaserScanPtr scan(new LaserScan());
  scan->ranges.push_back(0);

  EXPECT_CALL(validator, isValid(_));

  extractor.extractObstacleObservations(scan);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
