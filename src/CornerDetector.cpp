/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

using sensor_msgs::LaserScanPtr;

namespace selfie_obstacle_detection
{

CornerDetector::CornerDetector(IObstacleObservationsExtractor extractor)
	: extractor_(extractor) { }

void CornerDetector::detectCorners(LaserScanPtr scan) { }

} // namespace selfie_obstacle_detection
