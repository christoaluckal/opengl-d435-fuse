#include "model_draw.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  
int main(int argc, char **argv){
    std::vector< unsigned char > buffer = drawObj_s("cube.obj");
    cv::Mat img(768, 1024, CV_8UC3,&buffer[0]);
    cv::flip(img,img,0);
    cv::imwrite("test2.png", img);

}