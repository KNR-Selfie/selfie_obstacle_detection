/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

using sensor_msgs::LaserScanPtr;

namespace selfie_obstacle_detection
{

CornerDetector::CornerDetector(IObstacleObservationsExtractor* extractor,
                               ILineHelper* helper)
	: extractor_(extractor),
	  helper_   (helper)
{ }

void CornerDetector::detectCorners(LaserScanPtr scan)
{
	extractor_->extractObstacleObservations(scan);
}

} // namespace selfie_obstacle_detection
