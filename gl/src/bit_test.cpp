/*include necessary header files*/
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <chrono>
using namespace cv;
/*driver function: program execution begins from here*/

Mat bitwise(Mat image1,Mat image2)
{
    Mat output;
    bitwise_and(image1, image2, output);
    return output;
}

int main(){
    /*create two input arrays: matrices*/
    /*also populate matrices with zeros*/
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    Mat res;
    Mat image1 = imread("images/image_1.png"); 
    Mat image2 = imread("images/room.png"); 
    for(int i=0;i<1000;i++)
    {
        auto t1 = high_resolution_clock::now();
        res = bitwise(image1,image2);
        auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as a double. */
        duration<double, std::milli> ms_double = t2 - t1;

        std::cout << i << '\n';
        std::cout << ms_double.count() << "ms\n";
    }
    imwrite("out.png",res);
    return 0;
}
