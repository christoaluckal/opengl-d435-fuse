// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../helpers/example.hpp"          // Include short list of convenience functions for rendering
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>            // std::min, std::max
#include <fstream>              // std::ifstream

bool flag = false;
int main(int argc, char * argv[]) try
{
    // // Declare pointcloud object, for calculating pointclouds and texture mappings
    // rs2::pointcloud pc;
    // // We want the points object to be persistent so we can display the last cloud when a frame drops
    // rs2::points points;
    // store pose and timestamp
    rs2::pose_frame pose_frame(nullptr);
    // std::vector<rs2_vector> trajectory;

    rs2::context                          ctx;        // Create librealsense context for managing devices
    // std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));


    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        // colorizers[serial] = rs2::colorizer();
    }


    while (1) // Application still alive?
    {
        for (auto &&pipe : pipelines) // loop over pipelines
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
            if(flag==false)
            {
                cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
                cv::imwrite("test.png",image);
                flag = true;
            }
            
            // // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            // if (!color)
            //     color = frames.get_infrared_frame();

            // // Tell pointcloud object to map to this color frame
            // if (color)
            //     pc.map_to(color);

            auto depth = frames.get_depth_frame();

            // Generate the pointcloud and texture mappings
            // if (depth)
            //     points = pc.calculate(depth);

            // pose
            auto pose = frames.get_pose_frame();
            if (pose) {
                pose_frame = pose;

                // Print the x, y, z values of the translation, relative to initial position
                auto pose_data = pose.get_pose_data();
                std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
            }
            if(depth)
            {
                std::cout << "DIST:" << depth.get_distance(640 / 2, 480 / 2) << '\n';
            }
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}