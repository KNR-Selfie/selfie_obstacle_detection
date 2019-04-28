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
  for (int i = 0; i < scan->ranges.size(); i++)
  {
    ObstacleObservationPtr observation(new ObstacleObservation());

    while (i < scan->ranges.size() && validator_->isValid(scan->ranges[i]))
    {
      PointPtr point = transformer_->transformCoordinates(scan, i, scan->ranges[i]);
      observation->push_back(point);
      i++;
    }

    if (observation->size() > 0)
    {
      observations.push_back(observation);
    }
  }

  return observations;
}

}  // namespace selfie_obstacle_detection
