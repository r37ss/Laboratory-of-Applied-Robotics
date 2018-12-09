// Taken from: samples/cpp/tutorial_code/ImgProc/Smoothing/Smoothing.cpp
/**
 * file Smoothing.cpp
 * brief Sample code for simple filters
 * author OpenCV team
 */

#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

/// Global Variables
int DELAY_CAPTION = 1500;
int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31;

 Mat src; Mat dst;
// char window_name[] = "Smoothing Demo";

// /// Function headers
// int display_caption( const char* caption );
// int display_dst( int delay );


/**
 * function main
 */
int main(int argc, char* argv[] )
{   
    //string filename = argv[1];
    //Mat src;
    const char* filename = argc >=2 ? argv[1] : "../data/lena.jpg";
    src = imread( filename, IMREAD_COLOR );
    //cv::Mat img = cv::imread(filename.c_str());
    //namedWindow( window_name, WINDOW_NORMAL );
    //Display original image
    int i = 9;
    //cv::Mat dst=img.clone();
    
    medianBlur (src, dst, i );
    cv::imshow("Original", dst);
    cv::imwrite("Blla.jpg", dst);
    cv::waitKey(0);
    return 0;
}

