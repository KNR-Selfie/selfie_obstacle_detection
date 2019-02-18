/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/ObstacleObservationsExtractor.h"

using sensor_msgs::LaserScanPtr;

namespace selfie_obstacle_detection
{

ObstacleObservationsExtractor::ObstacleObservationsExtractor(IMeasurementValidator* validator)
	: validator_(validator)
{ }

ObstacleObservations ObstacleObservationsExtractor::extractObstacleObservations(LaserScanPtr scan)
{ }

} // namespace selfie_obstacle_detection
