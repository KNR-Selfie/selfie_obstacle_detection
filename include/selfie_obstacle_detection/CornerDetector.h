/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef CORNER_DETECTOR_H
#define CORNER_DETECTOR_H

#include "selfie_obstacle_detection/IObstacleObservationsExtractor.h"
#include "selfie_obstacle_detection/ILineHelper.h"
#include "selfie_obstacle_detection/ICornerGenerator.h"

#include <sensor_msgs/LaserScan.h>

namespace selfie_obstacle_detection
{

class CornerDetector
{
	IObstacleObservationsExtractor* extractor_;
	ILineHelper* helper_;
	ICornerGenerator* generator_;
public:
	CornerDetector(IObstacleObservationsExtractor* extractor, ILineHelper* helper, ICornerGenerator* generator);
	void detectCorners(sensor_msgs::LaserScanPtr scan);
}; // class CornerDetector

} // namespace selfie_obstacle_detection

#endif /* CORNER_DETECTOR_H */
