/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef OBSTACLE_OBSERVATIONS_EXTRACTOR_H
#define OBSTACLE_OBSERVATIONS_EXTRACTOR_H

#include "selfie_obstacle_detection/IObstacleObservationsExtractor.h"
#include "selfie_obstacle_detection/IMeasurementValidator.h"
#include "selfie_obstacle_detection/ICoordinatesTransformer.h"

namespace selfie_obstacle_detection
{

class ObstacleObservationsExtractor : public IObstacleObservationsExtractor
{
  IMeasurementValidator* validator_;
  ICoordinatesTransformer* transformer_;
public:
  ObstacleObservationsExtractor(IMeasurementValidator* validator, ICoordinatesTransformer* transformer);
  ObstacleObservations extractObstacleObservations(sensor_msgs::LaserScanPtr scan);
};  // class ObstacleObservationsExtractor

}  // namespace selfie_obstacle_detection

#endif /* OBSTACLE_OBSERVATIONS_EXTRACTOR_H */
