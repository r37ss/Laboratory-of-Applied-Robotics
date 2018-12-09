#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "Map.h"


Map::Map (cv::Mat image) : m_img_rgb(image), m_obstacles()
{

  cv::Mat img_hsv;
  cv::cvtColor(image, img_hsv, cv::COLOR_BGR2HSV);
  find_obstacles(img_hsv);

}


void Map::find_obstacles(cv::Mat image)
{
  //Color Mask
  cv::Mat low_hue_red_mask;
  cv::Mat high_hue_red_mask;
  cv::Mat red_mask;
  
  cv::inRange(image, cv::Scalar(LOW_H_R, LOW_S_R, LOW_V_R), cv::Scalar(M1_H_R, HIGH_S_R, HIGH_V_R), low_hue_red_mask);
  cv::inRange(image, cv::Scalar(M1_H_R, LOW_S_R, LOW_V_R), cv::Scalar(HIGH_H_R, HIGH_S_R, HIGH_V_R), high_hue_red_mask);
  cv::addWeighted(low_hue_red_mask, 1.0, high_hue_red_mask, 1.0, 0.0, red_mask); // combine together the two binary masks
  

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Point> approx_curve;
  cv::Rect rect;
      

  // Find contours and approximate in bounding boxes
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (int i=0; i<contours.size(); ++i)
  {
    if (contours[i].size() > MIN_OBSTACLE_CONTOUR_SIZE) { 
      approxPolyDP(contours[i], approx_curve, 10, true);

      rect = cv::boundingRect({approx_curve});
      cv::rectangle(image, rect, cv::Scalar(40,190,40), 2);
      Obstacle obstacle(rect);
      m_obstacles.push_back(obstacle);
    } 
  }

  imshow("path",image);
}



int main(int argc, char* argv[])
{

  cv::Mat img;
  img = cv::imread(argv[1], 1); //reading file
  
  if(img.empty()) {
    throw std::runtime_error("Failed to open the file ");
  }

  Map map(img);


  vector<Point2f> trajectory;
  trajectory = dubins(0, 0, (((double(-9) / double(2))) * PI), 600, 600, (PI / double(1)), 1);

  std::cout << trajectory << std::endl;

/*
  for (auto i : trajectory) {
    Point2i point = i;
    cv::circle(img, point, 2, cv::Scalar(40,190,40));
  }

  imshow("path",img);
  */waitKey(0);

}
