/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/ObstacleObservationsExtractor.h"

#include <gmock/gmock.h>

using selfie_obstacle_detection::IMeasurementValidator;
using selfie_obstacle_detection::ObstacleObservationsExtractor;

class MockMeasurementValidator
	: public IMeasurementValidator
{ };

TEST(ObstacleObservationsExtractorTestSuite, basicTest)
{
	MockMeasurementValidator validator;

	ObstacleObservationsExtractor extractor(&validator);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
