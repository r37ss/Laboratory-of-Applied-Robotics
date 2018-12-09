// ipm.cpp:
// Find the perspective mapping transformation from the ground floor to the
// camera, and store all the parameters to a file

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <atomic>
#include <unistd.h>

using namespace cv;
cv::Mat rectangular_points(4,2,CV_32F);

void loadCoefficients(const std::string& filename,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs)
{
  cv::FileStorage fs( filename, cv::FileStorage::READ );
  if (!fs.isOpened())
  {
    throw std::runtime_error("Could not open file " + filename);
  }
  fs["camera_matrix"] >> camera_matrix;
  fs["distortion_coefficients"] >> dist_coeffs;
  fs.release();
}


// Defintion of the function pickNPoints and the callback mouseCallback.
// The function pickNPoints is used to display a window with a background
// image, and to prompt the user to select n points on this image.
cv::Mat result, bg_img;
int idx = 0;
std::string name;
std::atomic<bool> done;
int n;

void mouseCallback(int event, int x, int y, int, void* p)
{
  if (event != EVENT_LBUTTONDOWN || done.load()) return;

  if (result.rows <= idx) throw std::runtime_error("Something went wrong");
  result.at<float>((int)idx, 0) = x;
  result.at<float>((int)idx++, 1) = y;
  cv::circle(bg_img, {x,y}, 20, Scalar(0,0,255), -1);
  imshow(name.c_str(), bg_img);

  if (idx >= n) {
    usleep(500*1000);
    done.store(true);
  }
}

cv::Mat pickNPoints(int n0, const Mat& img)
{
  result = cv::Mat(n0, 2, CV_32F);
  idx = 0;
  bg_img = img.clone();
  name = "Pick " + std::to_string(n0) + " points";
  imshow(name.c_str(), bg_img);
  namedWindow(name.c_str());
  n = n0;

  done.store(false);

  setMouseCallback(name.c_str(), &mouseCallback, nullptr);
  while (!done.load()) {
    cv::waitKey(500);
  }

  destroyWindow(name.c_str());
  return result;
}


// Example of function to determine the perspective transformation of a
// rectangle on the ground plane (with manual intervention from the user, that
// is required to select the 4 corner points of the rectangle, starting from the
// top-left corner and proceeding in clockwise order, and the origin (top-left)
// and scale of the transformed top view image).
// Since the real size of the rectangle is known (width: 1m, height: 1.5m),
// the fucntion returns also the pixel_scale, i.e. the size (in mm) of each
// pixel in the top view image
Mat findTransform(const std::string& calib_image_name,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs,
                  double& pixel_scale)
{
  Mat calib_image = imread(calib_image_name);

  // if (original_image.empty())
  // {
  //   throw std::runtime_error("Could not open image " + calib_image_name);
  // }

 // undistort(original_image, calib_image, camera_matrix, dist_coeffs);

  cv::Mat corner_pixels = rectangular_points.clone(); //pickNPoints(4, calib_image);

  // generate and show a black image, corresponding to the top-view camera, and
  // ask the user to select the origin and scale of the resulting image
  cv::Mat screen_pixels = Mat(calib_image.rows, calib_image.cols, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat dst_pixels(2,2,CV_32F); //= pickNPoints(2, screen_pixels);
  std::cout<<dst_pixels<<std::endl;
  std::cout<<"rows:"<<calib_image.rows<<std::endl;
  std::cout<<"cols:"<<calib_image.cols<<std::endl;
  dst_pixels.at<float>(0, 0) = 0;
  dst_pixels.at<float>(0, 1) = 0;

  dst_pixels.at<float>(1, 0) = 1024;
  dst_pixels.at<float>(1, 1) = 1280;

  float origin_x = dst_pixels.at<float>(0, 0),
        origin_y = dst_pixels.at<float>(0, 1);

  float delta_x = dst_pixels.at<float>(1, 0)-dst_pixels.at<float>(0, 0);
  float delta_y = dst_pixels.at<float>(1, 1)-dst_pixels.at<float>(0, 1);

  float delta_x_mm = 1000;
  float delta_y_mm = 1500;

  float scale_x = delta_x/delta_x_mm;
  float scale_y = delta_y/delta_y_mm;
  float scale = std::min(scale_x, scale_y);

  pixel_scale = 1./scale;
  delta_x = scale*delta_x_mm;
  delta_y = scale*delta_y_mm;

  cv::Mat transf_pixels = (cv::Mat_<float>(4,2) << origin_x, origin_y,
                                                   origin_x+delta_x, origin_y,
                                                   origin_x+delta_x, origin_y+delta_y,
                                                   origin_x, origin_y+delta_y);

  cv::Mat transf = cv::getPerspectiveTransform(corner_pixels, transf_pixels);
  cv::Mat unwarped_frame, concat;
  warpPerspective(calib_image, unwarped_frame, transf, calib_image.size());
  //cv::hconcat(calib_image, unwarped_frame, concat);
  imshow("Unwarping", unwarped_frame);
  imwrite("img1.jpg",unwarped_frame);

  waitKey(0);
  return transf;
}

// Store all the parameters to a file, for a later use, using the FileStorage
// class methods
void storeAllParameters(const std::string& filename,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        double pixel_scale,
                        const Mat& persp_transf)
{
  cv::FileStorage fs( filename, cv::FileStorage::WRITE );
  fs << "camera_matrix" << camera_matrix
     << "dist_coeffs" << dist_coeffs
     << "pixel_scale" << pixel_scale
     << "persp_transf" << persp_transf;
  fs.release();
}

void run(std::string filename) {
  cv::Mat camera_matrix, dist_coeffs;
  loadCoefficients("../config/intrinsic_calibration.xml", camera_matrix, dist_coeffs);

  double pixel_scale;
  Mat persp_transf = findTransform(filename, camera_matrix, dist_coeffs, pixel_scale);
  std::cout << "Pixel Scale: " << pixel_scale << "mm" << std::endl;

  storeAllParameters("../config/fullCalibration.yml", camera_matrix, dist_coeffs, pixel_scale, persp_transf);
}

// int main() {
  
//   return 0;
// }











// shape_detection.cpp:
// Detect shapes on a computer-generated image

static const int W_0      = 300;
static const int H_0      = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;


void processImage(std::string filename)
{
  // Load image from file
  //std::string filename = "imgs/polygons.png";
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
  //cv::imshow("HSV", hsv_img);
  //cv::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

  // Find black regions (filter on saturation and value)
  cv::Mat black_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 100), black_mask);  
  cv::imshow("BLACK_filter", black_mask);
  cv::moveWindow("BLACK_filter", W_0+2*(img.cols+OFFSET_W), H_0+img.rows+OFFSET_H);

  // Wait keypress
  cv::waitKey(0);

  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;

  // Process black mask
  cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
  contours_img = img.clone();
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    if (contours[i].size() > 200){
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 20, true); // fit a closed polygon (with less vertices) to the given contour,
                                                      // with an approximation accuracy (i.e. maximum distance between 
                                                     // the original and the approximated curve) of 3
    if(approx_curve.size()==4){
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;

    rectangular_points.at<float>(0,0) = approx_curve[0].x;
    rectangular_points.at<float>(0,1) = approx_curve[0].y;
    
    rectangular_points.at<float>(1,0) = approx_curve[3].x;
    rectangular_points.at<float>(1,1) = approx_curve[3].y;

    rectangular_points.at<float>(2,0) = approx_curve[2].x;
    rectangular_points.at<float>(2,1) = approx_curve[2].y;

    rectangular_points.at<float>(3,0) = approx_curve[1].x;
    rectangular_points.at<float>(3,1) = approx_curve[1].y;
    std::cout<<"ahole"<<std::endl;
    }
    }
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);
}
int main(int argc, char* argv[])
{
  processImage(argv[1]);
  run(argv[1]);
  return 0;
}
