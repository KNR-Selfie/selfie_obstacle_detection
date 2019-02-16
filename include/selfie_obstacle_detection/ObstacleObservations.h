/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef OBSTACLE_OBSERVATIONS_H
#define OBSTACLE_OBSERVATIONS_H

#include <vector>

namespace selfie_obstacle_detection
{

struct PointXY
{
	float x;
	float y;

	PointXY(float x, float y)
		: x(x), y(y) { };
};

struct Line
{
	float A;
	float B;
	float C;
};

typedef std::vector<PointXY> ObstacleObservation;
typedef std::vector<ObstacleObservation> ObstacleObservations;

} // namespace selfie_obstacle_detection

#endif /* OBSTACLE_OBSERVATIONS_H */
