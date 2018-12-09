// example01.cpp:
// Read an image from a file and display it on a window

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  Mat image;
  image = imread("img.jpg", IMREAD_COLOR); // Read the file
  if( image.empty() ) // Check for invalid input
  {
    cout << "Could not open or find the image" << std::endl ;
    return -1;
  }
  namedWindow("Win1", WINDOW_AUTOSIZE); // Create a window for display.
  imshow("Win1", image); // Show our image inside it.
  waitKey(0); // Wait for a keystroke
  return 0;
}
