// circle_detection.cpp:
// Detect circular shapes using Hough transform

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

static const int W_0      = 300;
static const int H_0      = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;


void processImage()
{
  // Load image from file
  std::string filename = "imgs/polygons.png";
  cv::Mat img = cv::imread(filename.c_str());
  if(img.empty()) {
    throw std::runtime_error("Failed to open the file " + filename);
  }
  
  // Display original image
  cv::imshow("Original", img);
  cv::moveWindow("Original", W_0, H_0);

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  
  // Display HSV image
  cv::imshow("HSV", hsv_img);
  cv::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(45, 10, 10), cv::Scalar(75, 255, 255), green_mask);
  cv::imshow("GREEN_filter", green_mask);
  cv::moveWindow("GREEN_filter", W_0+img.cols+OFFSET_W, H_0+img.rows+OFFSET_H);
  
  // Wait keypress
  cv::waitKey(0);

  // Blur image to improve performance of Hough detector
  cv::GaussianBlur(green_mask, green_mask, cv::Size(3, 3), 1, 1);
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(green_mask, circles, cv::HOUGH_GRADIENT, 1, 30, 50, 30, 0, 0); // Circle detection using Hough transform
  std::cout << "CIRCLES: " << circles.size() << std::endl;
  for( size_t i = 0; i < circles.size(); i++ )
  {
       cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
       int radius = cvRound(circles[i][2]);
       cv::circle( img, center, radius, cv::Scalar(0,170,220), 3, cv::LINE_AA, 0 ); // draw the circle outline
  }

  cv::imshow("Original", img);
  cv::waitKey(0);  

}

int main()
{
  processImage();
  return 0;
}
