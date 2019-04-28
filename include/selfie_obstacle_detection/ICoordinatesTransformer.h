/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_COORDINATES_TRANSFORMER_H
#define I_COORDINATES_TRANSFORMER_H

#include "selfie_obstacle_detection/ObstacleObservations.h"

#include <sensor_msgs/LaserScan.h>

namespace selfie_obstacle_detection
{

class ICoordinatesTransformer
{
public:
  virtual ~ICoordinatesTransformer() { }

  virtual PointPtr transformCoordinates(sensor_msgs::LaserScanPtr scan, int index, float value) = 0;
};  // class ICoordinatesTransformer

}  // namespace selfie_obstacle_detection

#endif /* I_COORDINATES_TRANSFORMER_H */
