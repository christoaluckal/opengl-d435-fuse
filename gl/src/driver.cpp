#include "../helpers/model_draw.h"
#include "../helpers/new_model_draw.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  
int main(int argc, char **argv){
    const char *path = argv[1];
    // Get Image from D435
    // Get coords of chair
    // Blacken the RGB where chair is
    // Calculate location of chair in room
    // drawObj("Chair.obj");
    actual("objs/Chair.obj");
    // drawObj(path);
    // std::vector< unsigned char > buffer = drawObj_s("cube.obj");
    // cv::Mat img(768, 1024, CV_8UC3,&buffer[0]);
    // cv::flip(img,img,0);
    // cv::imwrite("test2.png", img);

}