/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef OBSTACLE_OBSERVATIONS_EXTRACTOR_H
#define OBSTACLE_OBSERVATIONS_EXTRACTOR_H

#include "selfie_obstacle_detection/IObstacleObservationsExtractor.h"

namespace selfie_obstacle_detection
{

class ObstacleObservationsExtractor : public IObstacleObservationsExtractor
{
public:
	ObstacleObservations extractObstacleObservations(sensor_msgs::LaserScanPtr scan);
}; // class ObstacleObservationsExtractor

} // namespace selfie_obstacle_detection

#endif /* OBSTACLE_OBSERVATIONS_EXTRACTOR_H */
