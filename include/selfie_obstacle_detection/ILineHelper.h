/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_LINE_HELPER_H
#define I_LINE_HELPER_H

#include "selfie_obstacle_detection/ObstacleObservations.h"

namespace selfie_obstacle_detection
{

class ILineHelper
{
public:
  virtual ~ILineHelper() { }

  virtual bool fitLineToSegment(ObstacleObservation::iterator start,
                                ObstacleObservation::iterator end,
                                LinePtr& line) = 0;

  virtual PointPtr projectPointOntoLine(PointPtr point,
                                        LinePtr line) = 0;

  virtual bool arePerpendicular(LinePtr l1, LinePtr l2) = 0;

  virtual PointPtr findIntersection(LinePtr l1, LinePtr l2) = 0;
}; // class ILineHelper

} // namespace selfie_obstacle_detection

#endif /* I_LINE_HELPER_H */
