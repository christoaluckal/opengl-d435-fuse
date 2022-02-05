# opengl-d435-fuse-

The aim of this repo is to create an primitive AR system using OpenGL, Intel Realsense D435+T265 and YOLOv3 object detection. The object detection is implemented to detect chairs. The [result](https://github.com/christoaluckal/opengl-d435-fuse/blob/main/gl/output.mp4).

## Requirements
1. OpenGL
2. Intel Realsense SDK
3. OpenCV
4. YOLO model weights


### Limitations
1.  Only one OBJ file can be used
2.  Low framerate
3.  The overlapping of the real with the OpenGL model is not accurate
4.  The T265 initializes at 0,0,0 hence the model position needs to be offset compile-time to account for the difference in sizes
5.  The parameters of the real object (location, size) need to be estimated wrt the OBJ to provide proper alignment

In the `gl` folder run <br>
```g++ src/driver.cpp src/new_model_draw.cpp src/custom_obj_det.cpp ../common/*.cpp -lGL -lGLU -lGLEW -lglfw -lSOIL `pkg-config --cflags --libs opencv4` -lrealsense2```

### To DO
- [ ] Improve framerate <br>
- [ ] Allow keyboard input to allow fine-tuning of the model <br>
- [ ] Completely remove the real object and put only the OBJ <br>
