// example02.cpp:
// Read a video from a file and display it on a window

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  // VideoCapture captures the data from a video or an attached camera
  VideoCapture capture("video.avi"); // try to open video file

  // Check if video/camera was open successfully
  if (!capture.isOpened()) 
  {
    cout << "Could not open or find the video" << std::endl;
    return -1;
  } 

  namedWindow("Win1", WINDOW_AUTOSIZE); // create a new window
  Mat frame;
  while (true) {
    capture >> frame;
    if (frame.empty()) break; // No more frames available. Reached end of video
    imshow("Win1", frame); // Show captured frame on screen
    // Wait for a keyboard event, for up to N milliseconds. A call to waitKey is
    // required to allow HighGUI to process pending events and to redraw the active
    // windows. If N<=0, waitKeys waits indefinitely for a keyboard event.
    char key = (char)waitKey(30);
    if (key == 'q' || key == 27) break;
  }
}
