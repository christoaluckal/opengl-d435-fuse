// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// #include "../helpers/example.hpp"          // Include short list of convenience functions for rendering
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>            // std::min, std::max
#include <fstream>  
#include <iomanip>            // std::ifstream

#include<SOIL/SOIL.h>
// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

#include <GL/glut.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "../../common/shader.hpp"
#include "../../common/texture.hpp"
#include "../../common/controls.hpp"
#include "../../common/objloader.hpp"

#include <typeinfo>

#include "./custom_obj_det.hpp"
#include <chrono>

using clock_func = std::chrono::steady_clock;

std::string get_time(std::chrono::_V2::steady_clock::time_point time2,std::chrono::_V2::steady_clock::time_point time1)
{
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time2 - time1);
	return std::to_string(time_span.count());
}

int actual(const char *path)
{
	float real_angle=0;
	std::cin >> real_angle;
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( 640, 480, "Tutorial 07 - Model Loading", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, 640/2, 480/2);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Load the texture
	GLuint Texture = loadDDS("uvmap.DDS");
	
	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals; // Won't be used at the moment.
	bool res = loadOBJ(path, vertices, uvs, normals);

	// Load it into a VBO

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);
	glm::mat4 ProjectionMatrix;
	glm::mat4 ViewMatrix;
	glm::mat4 ModelMatrix = glm::mat4(1.0);
	std::vector< unsigned char > buf( 640 * 480 * 3 );


	rs2::pose_frame pose_frame(nullptr);
    // std::vector<rs2_vector> trajectory;

    rs2::context                          ctx;        // Create librealsense context for managing devices
    // std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

	if(serials.size()==0)
	{
		std::cout << "No Device\n";
		exit(0);
	}
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
	bool initial_test = true;
	int frame_count = 0;
	std::vector<std::pair<cv::Mat,cv::Mat>> image_pairs;

	auto curr = clock_func::now();
	auto start = clock_func::now();
	do{
		start = clock_func::now();
		buf.clear();
		rs2::depth_frame depth_f(nullptr);
		float yaw=0;
		float pitch=0;
		float posx,posz,posy;
		std::vector<int> bbox;
		cv::Mat image(cv::Size(640, 480), CV_8UC3, cv::Mat::AUTO_STEP);
		cv::Mat image2;
		for (auto &&pipe : pipelines) // loop over pipelines
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
			if(color)
			{
				cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
				// image = (void*)color.get_data();
				image2=image;
				cv::cvtColor(image2,image2,cv::COLOR_BGR2RGB);
				// std::memcpy(image.data, (void*)color.get_data(),640*480*3);
				start = std::chrono::steady_clock::now();
				bbox = customDetection(image2);
				curr = std::chrono::steady_clock::now();
				std::cout << "DETECTION:" << get_time(curr,start) << '\n';
				auto depth = frames.get_depth_frame();
				if(depth&&initial_test)
				{
					depth_f = depth.as<rs2::depth_frame>();
					// std::cout << "DIST:" << depth.get_distance(640 / 2, 480 / 2) << '\n';
				}
				continue;
			}
            
            // pose
            auto pose = frames.get_pose_frame();
            if (pose) {
                // Print the x, y, z values of the translation, relative to initial position
                auto pose_data = pose.as<rs2::pose_frame>().get_pose_data();
                auto w = pose_data.rotation.w;
				auto x = -pose_data.rotation.z;
				auto y = pose_data.rotation.x;
				auto z = -pose_data.rotation.y;

				auto pitch =  -asin(2.0 * (x*z - w*y)) * 180.0 / M_PI;
				// auto roll  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / M_PI;
				yaw   =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / M_PI;
				posx = pose_data.translation.x;
				posy = pose_data.translation.y;
				posz = pose_data.translation.z;
				// std::cout << posx << ' ' << posz << '\n';
            }

			// std::cout << "DIST:" << depth_f.get_distance(640 / 2, 480 / 2) << '\n';
        }
		curr = clock_func::now();
		std::cout << "REALSENSE LOOP:" << get_time(curr,start) << '\n';
		// // Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// Use our shader
		glUseProgram(programID);
		if(bbox[0]!=0&&bbox[2]!=0)
		{
			int x1,y1,x2,y2;
			x1 = bbox[0];
			y1 = bbox[1];
			x2 = bbox[2];
			y2 = bbox[3];
			float loc_x,loc_z;
			if(initial_test)
			{
				int sum_x = x1+x2;
				int y_off = int(y1+0.55*(y2-y1));
				loc_z = depth_f.get_distance(int(sum_x/2),int(y_off));
				loc_x = loc_z*tan(yaw*M_PI/180+(86/2)*((sum_x/2-320)/320));
				std::cout << loc_x << " " << posx+loc_x << " " << loc_z << " " << posz+loc_z << '\n';
				ModelMatrix = glm::rotate(ModelMatrix,glm::radians(real_angle),glm::vec3(0.0, 1.0, 0.0));
				ModelMatrix = glm::translate(ModelMatrix,glm::vec3(posx+loc_x, -1.0f, (posz+loc_z)));
				ModelMatrix = glm::scale(ModelMatrix,glm::vec3(0.047f));
				initial_test = false;
			}
			computeMatricesFromInputs_n(yaw,pitch,posx,posy,posz);
			ProjectionMatrix = getProjectionMatrix();
			ViewMatrix = getViewMatrix();
			// glm::mat4 ModelMatrix = glm::mat4(1.0);
			// ModelMatrix = glm::rotate(ModelMatrix,glm::radians(0.0f),glm::vec3(0.0, 1.0, 0.0));
			// ModelMatrix = glm::translate(ModelMatrix,glm::vec3(0.0f, 0.0f, 0.0f));
			// ModelMatrix = glm::scale(ModelMatrix,glm::vec3(scale));
			glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
			// std::cout << scale << '\n';
			// Send our transformation to the currently bound shader, 
			// in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

			// Bind our texture in Texture Unit 0
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, Texture);
			// Set our "myTextureSampler" sampler to use Texture Unit 0
			glUniform1i(TextureID, 0);

			// 1rst attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
			glVertexAttribPointer(
				0,                  // attribute
				3,                  // size
				GL_FLOAT,           // type
				GL_FALSE,           // normalized?
				0,                  // stride
				(void*)0            // array buffer offset
			);

			// 2nd attribute buffer : UVs
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribPointer(
				1,                                // attribute
				2,                                // size
				GL_FLOAT,                         // type
				GL_FALSE,                         // normalized?
				0,                                // stride
				(void*)0                          // array buffer offset
			);

			// Draw the triangle !
			glDrawArrays(GL_TRIANGLES, 0, vertices.size() );

			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(1);

			// Swap buffers
			glfwSwapBuffers(window);

			// glRotatef(0.0, 1.0, 0.0, 0.0);
			glReadPixels( 0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, &buf[0]);
			cv::Mat gl_img(480, 640, CV_8UC3,&buf[0]);
			cv::flip(gl_img,gl_img,0);

			// for(int row=bbox[1];row<bbox[3];row++)
			// {
			// 	for(int col=bbox[0];col<bbox[2];col++)
			// 	{
			// 		image2.at<cv::Vec3b>(row,col)[0]=0;
			// 		image2.at<cv::Vec3b>(row,col)[1]=0;
			// 		image2.at<cv::Vec3b>(row,col)[2]=0;
			// 	}
			// }
			image_pairs.push_back(std::make_pair(image2,gl_img));

			// cv::Mat res(cv::Size(640, 480), CV_8UC3);
			// std::string im_name,gl_name,res_name;
			// res_name = "res/file_"+std::to_string(frame_count)+".png";
			// im_name = "res/im_"+std::to_string(frame_count)+".png";
			// cv::bitwise_or(gl_img,image2,res);
			// // cv::cvtColor(res, res, cv::COLOR_BGR2RGB);
			// cv::imwrite(res_name,res);
			// frame_count+=1;
			glfwPollEvents();
		}
		
		// Compute the MVP matrix from keyboard and mouse input

	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// std::cout << "SAVING IMAGES\n";
	// for(int p=0;p<image_pairs.size();p++)
	// {
	// 	cv::Mat res(cv::Size(640, 480), CV_8UC3);
	// 	cv::bitwise_or(image_pairs[p].first,image_pairs[p].second,res);
	// 	cv::cvtColor(res, res, cv::COLOR_BGR2RGB);
	// 	cv::imwrite("res/file_"+std::to_string(p)+".png",res);

	// }

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}
// ffmpeg -framerate 10 -i res/file_%d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4


int nmd_steps(const char *path)
{
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( 1024, 768, "Tutorial 07 - Model Loading", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, 1024/2, 768/2);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Load the texture
	GLuint Texture = loadDDS("uvmap.DDS");
	
	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals; // Won't be used at the moment.
	bool res = loadOBJ(path, vertices, uvs, normals);

	// Load it into a VBO

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

	// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);

		// Compute the MVP matrix from keyboard and mouse input
		double scale = computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();
		glm::mat4 ModelMatrix = glm::mat4(1.0);
		ModelMatrix = glm::rotate(ModelMatrix,glm::radians(0.0f),glm::vec3(0.0, 1.0, 0.0));
		ModelMatrix = glm::translate(ModelMatrix,glm::vec3(0.0f, 0.0f, 0.0f));
		ModelMatrix = glm::scale(ModelMatrix,glm::vec3(scale));
		glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
		std::cout << scale << '\n';
		// Send our transformation to the currently bound shader, 
		// in the "MVP" uniform
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

		// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, Texture);
		// Set our "myTextureSampler" sampler to use Texture Unit 0
		glUniform1i(TextureID, 0);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attribute buffer : UVs
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		glVertexAttribPointer(
			1,                                // attribute
			2,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, vertices.size() );

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

		// Swap buffers
		glfwSwapBuffers(window);
		std::vector< unsigned char > buf( 1024 * 768 * 3 );
		// glRotatef(270.0, 1.0, 0.0, 0.0);
		glReadPixels( 0, 0, 1024, 768, GL_RGB, GL_UNSIGNED_BYTE, &buf[0]);

        // int err = SOIL_save_image
        //     (
        //     "images/soil_img.bmp",
        //     SOIL_SAVE_TYPE_BMP,
        //     1024, 768, 3,
        //     &buf[0]
        //     );
		glfwPollEvents();

	// for(int i=0;i<100;i++){

	// 	// // Clear the screen
	// 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 	// Use our shader
	// 	// glUseProgram(programID);

	// 	// Compute the MVP matrix from keyboard and mouse input
	// 	double scale = computeMatricesFromInputs();
	// 	glm::mat4 ProjectionMatrix = getProjectionMatrix();
	// 	glm::mat4 ViewMatrix = getViewMatrix();
	// 	// glm::mat4 ModelMatrix = glm::mat4(1.0);
	// 	ModelMatrix = glm::rotate(ModelMatrix,glm::radians(0.0f),glm::vec3(0.0, 1.0, 0.0));
	// 	ModelMatrix = glm::translate(ModelMatrix,glm::vec3(0.0f, 0.0f, 0.0f));
	// 	ModelMatrix = glm::scale(ModelMatrix,glm::vec3(scale));
	// 	glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
	// 	// std::cout << scale << '\n';
	// 	// Send our transformation to the currently bound shader, 
	// 	// in the "MVP" uniform
	// 	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

	// 	// Bind our texture in Texture Unit 0
	// 	glActiveTexture(GL_TEXTURE0);
	// 	glBindTexture(GL_TEXTURE_2D, Texture);
	// 	// Set our "myTextureSampler" sampler to use Texture Unit 0
	// 	glUniform1i(TextureID, 0);

	// 	// 1rst attribute buffer : vertices
	// 	glEnableVertexAttribArray(0);
	// 	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	// 	glVertexAttribPointer(
	// 		0,                  // attribute
	// 		3,                  // size
	// 		GL_FLOAT,           // type
	// 		GL_FALSE,           // normalized?
	// 		0,                  // stride
	// 		(void*)0            // array buffer offset
	// 	);

	// 	// 2nd attribute buffer : UVs
	// 	glEnableVertexAttribArray(1);
	// 	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	// 	glVertexAttribPointer(
	// 		1,                                // attribute
	// 		2,                                // size
	// 		GL_FLOAT,                         // type
	// 		GL_FALSE,                         // normalized?
	// 		0,                                // stride
	// 		(void*)0                          // array buffer offset
	// 	);

	// 	// Draw the triangle !
	// 	glDrawArrays(GL_TRIANGLES, 0, vertices.size() );

	// 	glDisableVertexAttribArray(0);
	// 	glDisableVertexAttribArray(1);

	// 	// Swap buffers
	// 	glfwSwapBuffers(window);
	// 	// std::vector< unsigned char > buf( 1024 * 768 * 3 );
	// 	glRotatef(0.0, 1.0, 0.0, 0.0);
	// 	glReadPixels( 0, 0, 1024, 768, GL_RGB, GL_UNSIGNED_BYTE, &buf[0]);

	// 	glfwPollEvents();

	// } // Check if the ESC key was pressed or the window was closed
	// // while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
	// // 	   glfwWindowShouldClose(window) == 0 );
	
        int err = SOIL_save_image
            (
            "images/soil_img.bmp",
            SOIL_SAVE_TYPE_BMP,
            1024, 768, 3,
            &buf[0]
            );

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

int nmd_drawObj(const char *path )
{
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( 1024, 768, "Tutorial 07 - Model Loading", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, 1024/2, 768/2);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Load the texture
	GLuint Texture = loadDDS("uvmap.DDS");
	
	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals; // Won't be used at the moment.
	bool res = loadOBJ(path, vertices, uvs, normals);

	// Load it into a VBO

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

	do{

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);

		// Compute the MVP matrix from keyboard and mouse input
		double scale = computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();
		glm::mat4 ModelMatrix = glm::mat4(1.0);
		ModelMatrix = glm::rotate(ModelMatrix,glm::radians(0.0f),glm::vec3(0.0, 1.0, 0.0));
		ModelMatrix = glm::translate(ModelMatrix,glm::vec3(0.0f, 0.0f, 0.0f));
		ModelMatrix = glm::scale(ModelMatrix,glm::vec3(scale));
		glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
		std::cout << scale << '\n';
		// Send our transformation to the currently bound shader, 
		// in the "MVP" uniform
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

		// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, Texture);
		// Set our "myTextureSampler" sampler to use Texture Unit 0
		glUniform1i(TextureID, 0);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);

		// 2nd attribute buffer : UVs
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		glVertexAttribPointer(
			1,                                // attribute
			2,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
		);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, vertices.size() );

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

		// Swap buffers
		glfwSwapBuffers(window);
		std::vector< unsigned char > buf( 1024 * 768 * 3 );
		glRotatef(180.0, 1.0, 0.0, 0.0);
		glReadPixels( 0, 0, 1024, 768, GL_RGB, GL_UNSIGNED_BYTE, &buf[0]);

        int err = SOIL_save_image
            (
            "img.bmp",
            SOIL_SAVE_TYPE_BMP,
            1024, 768, 3,
            &buf[0]
            );
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

std::vector< unsigned char > drawObj_s(const char *path )
{
	std::vector< unsigned char > buf(1024 * 768 * 3 );
	// Initialise GLFW
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return buf;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( 1024, 768, "Tutorial 07 - Model Loading", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return buf;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return buf;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, 1024/2, 768/2);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS); 

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "TextureFragmentShader.fragmentshader" );

	// Get a handle for our "MVP" uniform
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");

	// Load the texture
	GLuint Texture = loadDDS("uvmap.DDS");
	
	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals; // Won't be used at the moment.
	bool res = loadOBJ(path, vertices, uvs, normals);

	// Load it into a VBO

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

	// Clear the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Use our shader
	glUseProgram(programID);

	// Compute the MVP matrix from keyboard and mouse input
	computeMatricesFromInputs();
	glm::mat4 ProjectionMatrix = getProjectionMatrix();
	glm::mat4 ViewMatrix = getViewMatrix();
	glm::mat4 ModelMatrix = glm::mat4(1.0);
	glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture);
	// Set our "myTextureSampler" sampler to use Texture Unit 0
	glUniform1i(TextureID, 0);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexAttribPointer(
		0,                  // attribute
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	// 2nd attribute buffer : UVs
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glVertexAttribPointer(
		1,                                // attribute
		2,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, vertices.size() );

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	// Swap buffers
	glfwSwapBuffers(window);
	// std::vector< unsigned char > buf( 1024 * 768 * 3 );
	// glRotatef(180.0, 1.0, 1.0, 1.0);
	glReadPixels( 0, 0, 1024, 768, GL_BGR, GL_UNSIGNED_BYTE, &buf[0]);
	// int err = SOIL_save_image
	// 	(
	// 	"img.bmp",
	// 	SOIL_SAVE_TYPE_BMP,
	// 	1024, 768, 3,
	// 	&buf[0]
	// 	);
	glfwHideWindow(window);
	glfwPollEvents();
 // Check if the ESC key was pressed or the window was closed
	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return buf;
}

