/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include "selfie_obstacle_detection/CornerDetector.h"

using sensor_msgs::LaserScanPtr;
using std::prev;

namespace selfie_obstacle_detection
{

CornerDetector::CornerDetector(IObstacleObservationsExtractor* extractor,
                               ILineHelper* helper,
                               ICornerGenerator* generator)
  : extractor_(extractor),
    helper_(helper),
    generator_(generator)
{ }

CornerArrayPtr CornerDetector::detectCorners(LaserScanPtr scan)
{
  // generate obstacles candidates of incoming scan
  ObstacleObservations observations = extractor_->extractObstacleObservations(scan);
  // generate corners to return
  CornerArrayPtr corners(new CornerArray());
  for (auto observation : observations)
  {
    LinePtr rightLine;
    //begin of observation iterator
    auto cornerPointLocation = observation->begin();

    /**
     * Find the largest right subrange of points that can be fit by a line.
     * Starting from first point to end, first point +1 to end until line bend point
     * Generating ritght line until getting good accuracy according to threshold
     * Exit generating if good line fits well
     */
    while (cornerPointLocation < observation->end())
    {
      if (helper_->fitLineToSegment(cornerPointLocation, observation->end(), rightLine)) break; // generuje right line i szuka punktu załamania krzywej jezeli widoczne sa 2 boki
      cornerPointLocation++;
    }

    /**
     * CASE 1: All points can be fit by a single line.
     *         We see one edge of the obstacle.
     * Generating 2 corners
     */
    if (cornerPointLocation == observation->begin())
    {
      PointPtr p1 = helper_->projectPointOntoLine(*observation->begin(), rightLine);
      PointPtr p2 = helper_->projectPointOntoLine(*prev(observation->end()), rightLine);

      CornerPtr c1, c2;
      generator_->generateCorners(p1, p2, c1, c2);

      corners->data.push_back(*c1);
      corners->data.push_back(*c2);

      continue;
    }

    LinePtr leftLine;

    /**
     * CASE 2: Points form two subsets, each of which can be fit by a line.
     *         We might be seeing a corner of the obstacle.
     * Find another line of obstacle
     */
    if (helper_->fitLineToSegment(observation->begin(), prev(cornerPointLocation), leftLine))// poprzedni punkt, ponieważ wierzchołek należy tylko do prawej ściany, wiec chcecemy wziac punt poprzedni ktory jest kandydatem na ostatni punkt sciany lewej
    {
      // Proceed only if the lines do indeed form a right-angled corner
      if (helper_->arePerpendicular(leftLine, rightLine))
      {
        PointPtr p1 = helper_->projectPointOntoLine(*observation->begin(), leftLine);
        PointPtr p2 = helper_->findIntersection(leftLine, rightLine);
        PointPtr p3 = helper_->projectPointOntoLine(*prev(observation->end()), rightLine);

        CornerPtr c1, c2, c3;
        generator_->generateCorners(p1, p2, p3, c1, c2, c3);

        corners->data.push_back(*c1);
        corners->data.push_back(*c2);
        corners->data.push_back(*c3);
      }

      continue;
    }
  }

  corners->header = scan->header;
  return corners;
}

}  // namespace selfie_obstacle_detection
