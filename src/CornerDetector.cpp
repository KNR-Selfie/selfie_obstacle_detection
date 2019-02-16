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
	ObstacleObservations observations = extractor_->extractObstacleObservations(scan);

	for (auto observation : observations)
	{
		LinePtr line;
		helper_->fitLineToSegment(observation.begin(), observation.end(), line);

		helper_->projectPointOntoLine(*observation.begin(), line);
		helper_->projectPointOntoLine(*std::prev(observation.end()), line);
	}
}

} // namespace selfie_obstacle_detection
