#include "Obstacle.h"

Obstacle::Obstacle (cv::Rect rect) : m_bbox(rect) { }

cv::Rect Obstacle::get_bounding_box () {
  return m_bbox;
}