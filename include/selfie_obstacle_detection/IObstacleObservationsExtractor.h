/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_OBSTACLE_OBSERVATIONS_EXTRACTOR_H
#define I_OBSTACLE_OBSERVATIONS_EXTRACTOR_H

#include <sensor_msgs/LaserScan.h>

namespace selfie_obstacle_detection
{

typedef int ObstacleObservations;

class IObstacleObservationsExtractor
{
public:
	virtual ~IObstacleObservationsExtractor() {}
	virtual ObstacleObservations extractObstacleObservations(sensor_msgs::LaserScanPtr scan) = 0;
}; // class IObstacleObservationsExtractor

} // namespace selfie_obstacle_detection

#endif /* I_OBSTACLE_OBSERVATIONS_EXTRACTOR_H */
