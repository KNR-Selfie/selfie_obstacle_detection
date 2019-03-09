/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef OBSTACLE_OBSERVATIONS_H
#define OBSTACLE_OBSERVATIONS_H

#include <memory>
#include <vector>

namespace selfie_obstacle_detection
{

struct Point
{
  float x;
  float y;

  Point(float x, float y)
    : x(x), y(y) { };
};

typedef std::shared_ptr<Point> PointPtr;

struct Line
{
  float A;
  float B;
  float C;
};

typedef std::shared_ptr<Line> LinePtr;

typedef std::vector<PointPtr> ObstacleObservation;
typedef std::shared_ptr<ObstacleObservation> ObstacleObservationPtr;

typedef std::vector<ObstacleObservationPtr> ObstacleObservations;

} // namespace selfie_obstacle_detection

#endif /* OBSTACLE_OBSERVATIONS_H */
