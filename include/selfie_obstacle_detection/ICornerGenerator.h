/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_CORNER_GENERATOR_H
#define I_CORNER_GENERATOR_H

#include "selfie_obstacle_detection/ObstacleObservations.h"
#include "selfie_obstacle_detection/Corner.h"

namespace selfie_obstacle_detection
{

class ICornerGenerator
{
public:
	virtual ~ICornerGenerator() { }

	virtual void generateCorners(PointPtr   p1, PointPtr   p2,
	                             CornerPtr& c1, CornerPtr& c2) = 0;

	virtual void generateCorners(PointPtr   p1, PointPtr   p2, PointPtr   p3,
	                             CornerPtr& c1, CornerPtr& c2, CornerPtr& c3) = 0;
}; // class ICornerGenerator

} // namespace selfie_obstacle_detection

#endif /* I_CORNER_GENERATOR_H */
