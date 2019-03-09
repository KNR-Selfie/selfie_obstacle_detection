/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/ObstacleObservationsExtractor.h"

using sensor_msgs::LaserScanPtr;

namespace selfie_obstacle_detection
{

ObstacleObservationsExtractor::ObstacleObservationsExtractor(IMeasurementValidator* validator,
    ICoordinatesTransformer* transformer)
  : validator_(validator),
    transformer_(transformer)
{ }

ObstacleObservations ObstacleObservationsExtractor::extractObstacleObservations(LaserScanPtr scan)
{
  ObstacleObservations observations;

  for (auto it = scan->ranges.begin(); it < scan->ranges.end(); it++)
  {
    validator_->isValid(*it);
  }

  return observations;
}

} // namespace selfie_obstacle_detection
