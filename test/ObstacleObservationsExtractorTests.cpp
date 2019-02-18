/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/ObstacleObservationsExtractor.h"

#include <gmock/gmock.h>

using selfie_obstacle_detection::ObstacleObservationsExtractor;

TEST(ObstacleObservationsExtractorTestSuite, basicTest)
{
	ObstacleObservationsExtractor extractor;
}

int main(int argc, char **argv)
{
	::testing::InitGoogleMock(&argc, argv);
	return RUN_ALL_TESTS();
}
