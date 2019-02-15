/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef CORNER_DETECTOR_H
#define CORNER_DETECTOR_H

#include "selfie_obstacle_detection/IObstacleObservationsExtractor.h"

#include <sensor_msgs/LaserScan.h>

namespace selfie_obstacle_detection
{

class CornerDetector
{
	IObstacleObservationsExtractor* extractor_;
public:
	CornerDetector(IObstacleObservationsExtractor* extractor);
	void detectCorners(sensor_msgs::LaserScanPtr scan);
}; // class CornerDetector

} // namespace selfie_obstacle_detection

#endif /* CORNER_DETECTOR_H */
