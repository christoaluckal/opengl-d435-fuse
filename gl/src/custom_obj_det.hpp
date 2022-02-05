#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;

inline void preprocess(const Mat& frame, cv::dnn::dnn4_v20211220::Net& net, Size inpSize, float scale,const Scalar& mean, bool swapRB);
std::vector<int> postprocess(Mat& frame, const std::vector<Mat>& outs, cv::dnn::dnn4_v20211220::Net& net, int backend);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
void callback(int pos, void* userdata);

std::vector<int> getCoords(Mat img_frame);
std::vector<int> customDetection(Mat img1);