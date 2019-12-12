/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#include <selfie_obstacle_detection/RemoveBackground.h>

namespace selfie_obstacle_detection
{
RemoveBackgroundNodelet::RemoveBackgroundNodelet()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // Read parameters
  private_nh.param("low_threshold",  low_threshold_, -0.25f); // skok wymagany do zaliczenia początku przeszkody
  private_nh.param("high_threshold", high_threshold_, 0.25f); // skok wymagany do zakończenia przeszkody
  private_nh.param("min_size",       min_size_,       0.075f); //minimalna długośc boku przeszkody
  private_nh.param("max_size",       max_size_,       0.5f); // maksymalna długość przekątnej przeszkody

  // Subscribe to incoming scans and republish after filtration
  sub_ = nh.subscribe("scan", 10, &RemoveBackgroundNodelet::processScan, this);
  pub_ = nh.advertise<LaserScan>("scan_filtered", 10);
}

inline void appendZeros(std::vector<float>& v, int n)
{
  for (int i = 0; i < n; i++) v.push_back(0);
}

inline LaserScanPtr constructNewMessage(const LaserScanPtr in_scan)
{
  LaserScanPtr out_scan(new LaserScan);

  out_scan->header          = in_scan->header;

  out_scan->angle_min       = in_scan->angle_min;
  out_scan->angle_max       = in_scan->angle_max;
  out_scan->angle_increment = in_scan->angle_increment;

  out_scan->time_increment  = in_scan->time_increment;
  out_scan->scan_time       = in_scan->scan_time;

  out_scan->range_min       = in_scan->range_min;
  out_scan->range_max       = in_scan->range_max;

  out_scan->intensities     = in_scan->intensities;

  return out_scan;
}

void RemoveBackgroundNodelet::processScan(const LaserScanPtr& in_scan)
{
  ROS_INFO("NEW SCAN");
  
  int n_readings = in_scan->ranges.size();

  // We aren't supposed to modify the incoming message,
  // so we need to create a new one to be republished
  LaserScanPtr out_scan = constructNewMessage(in_scan);
  
  bool interesting_segment = false;
  int start = 0;
  for (int i = 1; i < n_readings - 1; i++)
  {
    float range_diff = in_scan->ranges[i + 1] - in_scan->ranges[i - 1];
    //  przy przejsciu z dalszego punktu na blizszy (poczatek przeszkody) roznica jest na minusie dlatego lower threshold jest progiem wejścia na minusie
    if (range_diff < low_threshold_)
    {
      ROS_INFO("Low diff: %f m", range_diff);

      // Fill with zeros for [start, i)
      appendZeros(out_scan->ranges, i - start);

      interesting_segment = true;
      start = i;
      continue;
    }
    // przy wyjściu z przeszkody otrzymujemy skany o dalszej odleglosci, wiec jest to wartosc progowa okreslajaca czy roznica nie jest na tyle duza ze interesujacy segment sie nie skonczyl
    if (range_diff > high_threshold_ && interesting_segment)
    {
      ROS_INFO("High diff: %f m", range_diff);

      float start_angle = in_scan->angle_min + start * in_scan->angle_increment;
      float stop_angle  = in_scan->angle_min + i     * in_scan->angle_increment;

      float start_x = in_scan->ranges[start] * std::cos(start_angle);
      float start_y = in_scan->ranges[start] * std::sin(start_angle);

      float stop_x = in_scan->ranges[i] * std::cos(stop_angle);
      float stop_y = in_scan->ranges[i] * std::sin(stop_angle);
      // sprawdzamy czy odleglosc przeszkody jest sensowna 
      // gdy widzimy jeden bok to liczymy jego bok,(minimum), albo przekątna przeszkody
      float segment_size = std::sqrt(std::pow(stop_x - start_x, 2)
                                     + std::pow(stop_y - start_y, 2));

      ROS_INFO("Start: %f rad, %f m", start_angle, in_scan->ranges[start]);
      ROS_INFO("Stop:  %f rad, %f m", stop_angle,  in_scan->ranges[i]);
      ROS_INFO("Size: %f m", segment_size);
      // sprawdzenie czy wielkosc boku nie jest mniejsza niz najmniejszy bok okreslony w zawodach lub wieksza niz przekatna przeszkody
      // jezeli miesci sie w odchylkach to dodawane sa punkty 
      // jezeli nie miesci sie to skan uzupelniany jest 0
      if (min_size_ <= segment_size && segment_size <= max_size_)
      {
        // Preserve current scan segment as it is a good obstacle candidate
        // Copy range values from source scan for [start, i]
        for (int j = start; j < i + 1; j++) out_scan->ranges.push_back(in_scan->ranges[j]);
      }
      else
      {
        // Fill with zeros for [start, i]
        appendZeros(out_scan->ranges, i + 1 - start);
      }

      interesting_segment = false;
      start = i + 1;
    }
  }

  // ??????
  // Pad with zeros
  appendZeros(out_scan->ranges, n_readings - start);

  // Republish filtered scan
  pub_.publish(out_scan); // zwraca wspolrzedne lidaru (kat, distance)
}

}  // namespace selfie_obstacle_detection

