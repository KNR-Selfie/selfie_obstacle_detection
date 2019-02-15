/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_OBSTACLE_OBSERVATIONS_EXTRACTOR_H
#define I_OBSTACLE_OBSERVATIONS_EXTRACTOR_H

#include <sensor_msgs/LaserScan.h>

namespace selfie_obstacle_detection
{

class IObstacleObservationsExtractor
{
public:
	virtual ~IObstacleObservationsExtractor() {}
}; // class IObstacleObservationsExtractor

} // namespace selfie_obstacle_detection

#endif /* I_OBSTACLE_OBSERVATIONS_EXTRACTOR_H */
