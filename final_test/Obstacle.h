#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <opencv2/core.hpp>

class Obstacle
{
  private:
    cv::Rect m_bbox;

  public:
    Obstacle(cv::Rect rect);
    cv::Rect get_bounding_box();
};
#endif