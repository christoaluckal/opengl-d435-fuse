#include<vector>
#include<string>
#include<chrono>
#include "../src/yolo_net.hpp"
int actual(const char *path);
int nmd_drawObj(const char *path );
int nmd_steps(const char *path);
std::vector< unsigned char > drawObj_s(const char *path );
std::string get_time(std::chrono::_V2::steady_clock::time_point time1,std::chrono::_V2::steady_clock::time_point time2);
CustomYoloNet createNet();
// void display();