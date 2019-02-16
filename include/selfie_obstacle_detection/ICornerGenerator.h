/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_CORNER_GENERATOR_H
#define I_CORNER_GENERATOR_H

#include "selfie_obstacle_detection/ObstacleObservations.h"

namespace selfie_obstacle_detection
{

class ICornerGenerator
{
public:
	virtual ~ICornerGenerator() { }
}; // class ICornerGenerator

} // namespace selfie_obstacle_detection

#endif /* I_CORNER_GENERATOR_H */
