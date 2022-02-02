#include <opencv2/opencv.hpp>
// g++ image.cpp `pkg-config --cflags --libs opencv4`
using namespace cv;
int main(int argc, char** argv) 
{ 
 // Read the image file 
 Mat image1 = imread("ref.png"); 
 return 0; 
}