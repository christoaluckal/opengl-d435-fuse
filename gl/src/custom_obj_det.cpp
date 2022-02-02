#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#if defined(CV_CXX11) && defined(HAVE_THREADS)
#define USE_THREADS 1
#endif
#ifdef USE_THREADS
#include <mutex>
#include <thread>
#include <queue>
#endif
#include "../helpers/common.hpp"
using namespace cv;
using namespace dnn;
float confThreshold, nmsThreshold;
std::vector<std::string> classes;
inline void preprocess(const Mat& frame, Net& net, Size inpSize, float scale,
                       const Scalar& mean, bool swapRB);
std::vector<int> postprocess(Mat& frame, const std::vector<Mat>& out, Net& net, int backend);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
void callback(int pos, void* userdata);

std::vector<int> getCoords(){
    confThreshold = 0.5;
    nmsThreshold = 0.4;
    float scale = 0.00392;
    Scalar mean = Scalar(0,0,0,0);
    bool swapRB = true;
    int inpWidth = 640;
    int inpHeight = 480;
    size_t asyncNumReq = 0;
    // CV_Assert(parser.has("model"));
    std::string modelPath = "detection/yolov3.weights";
    std::string configPath = "detection/darknet/cfg/yolov3.cfg";
    // Open file with classes names.
    std::string file = "object_detection_classes_pascal_voc.txt";
    std::ifstream ifs(file.c_str());
    if (!ifs.is_open())
        CV_Error(Error::StsError, "File " + file + " not found");
    std::string line;
    while (std::getline(ifs, line))
    {
        classes.push_back(line);
    }

    // Load a model.
    Net net = readNet(modelPath, configPath,"");
    int backend = 0;
    net.setPreferableBackend(backend);
    net.setPreferableTarget(0);
    std::vector<String> outNames = net.getUnconnectedOutLayersNames();
    // Create a window
    static const std::string kWinName = "Deep learning object detection in OpenCV";
    namedWindow(kWinName, WINDOW_NORMAL);
    int initialConf = (int)(confThreshold * 100);
    createTrackbar("Confidence threshold, %", kWinName, &initialConf, 99, callback);
    // Open a video file or an image file or a camera stream.
    // VideoCapture cap;
    // if (parser.has("input"))
    //     cap.open(parser.get<String>("input"));
    // else
    //     cap.open(parser.get<int>("device"));
#ifdef USE_THREADS
    bool process = true;
    // Frames capturing thread
    QueueFPS<Mat> framesQueue;
    std::thread framesThread([&](){
        Mat frame;
        while (process)
        {
            cap >> frame;
            if (!frame.empty())
                framesQueue.push(frame.clone());
            else
                break;
        }
    });
    // Frames processing thread
    QueueFPS<Mat> processedFramesQueue;
    QueueFPS<std::vector<Mat> > predictionsQueue;
    std::thread processingThread([&](){
        std::queue<AsyncArray> futureOutputs;
        Mat blob;
        while (process)
        {
            // Get a next frame
            Mat frame;
            {
                if (!framesQueue.empty())
                {
                    frame = framesQueue.get();
                    if (asyncNumReq)
                    {
                        if (futureOutputs.size() == asyncNumReq)
                            frame = Mat();
                    }
                    else
                        framesQueue.clear();  // Skip the rest of frames
                }
            }
            // Process the frame
            if (!frame.empty())
            {
                preprocess(frame, net, Size(inpWidth, inpHeight), scale, mean, swapRB);
                processedFramesQueue.push(frame);
                if (asyncNumReq)
                {
                    futureOutputs.push(net.forwardAsync());
                }
                else
                {
                    std::vector<Mat> outs;
                    net.forward(outs, outNames);
                    predictionsQueue.push(outs);
                }
            }
            while (!futureOutputs.empty() &&
                   futureOutputs.front().wait_for(std::chrono::seconds(0)))
            {
                AsyncArray async_out = futureOutputs.front();
                futureOutputs.pop();
                Mat out;
                async_out.get(out);
                predictionsQueue.push({out});
            }
        }
    });
    // Postprocessing and rendering loop
    while (waitKey(1) < 0)
    {
        if (predictionsQueue.empty())
            continue;
        std::vector<Mat> outs = predictionsQueue.get();
        Mat frame = processedFramesQueue.get();
        postprocess(frame, outs, net, backend);
        if (predictionsQueue.counter > 1)
        {
            std::string label = format("Camera: %.2f FPS", framesQueue.getFPS());
            putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
            label = format("Network: %.2f FPS", predictionsQueue.getFPS());
            putText(frame, label, Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
            label = format("Skipped frames: %d", framesQueue.counter - predictionsQueue.counter);
            putText(frame, label, Point(0, 45), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
        }
        imshow(kWinName, frame);
    }
    process = false;
    framesThread.join();
    processingThread.join();
#else  // USE_THREADS
    if (asyncNumReq)
        CV_Error(Error::StsNotImplemented, "Asynchronous forward is supported only with Inference Engine backend.");
    // Process frames.
    Mat frame, blob;
    // GET FRAME HERE
    frame = imread("test_chair.png");
    preprocess(frame, net, Size(inpWidth, inpHeight), scale, mean, swapRB);
    std::vector<Mat> outs;
    net.forward(outs, outNames);
    std::vector<int> box_coord = postprocess(frame, outs, net, backend);
    // Put efficiency information.
    // std::vector<double> layersTimes;
    // double freq = getTickFrequency() / 1000;
    // double t = net.getPerfProfile(layersTimes) / freq;
    // std::string label = format("Inference time: %.2f ms", t);
    // putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
    // std::cout << box_coord[0] << ' ' << box_coord[1] << ' ' << box_coord[2] << ' ' << box_coord[3] << '\n';
    // while (waitKey(1) < 0)
    // {
    //     // cap >> frame;
    //     if (frame.empty())
    //     {
    //         waitKey();
    //         break;
    //     }

    //     imshow(kWinName, frame);
    // }
#endif  // USE_THREADS
    return box_coord;
};

#ifdef USE_THREADS
template <typename T>
class QueueFPS : public std::queue<T>
{
public:
    QueueFPS() : counter(0) {}
    void push(const T& entry)
    {
        std::lock_guard<std::mutex> lock(mutex);
        std::queue<T>::push(entry);
        counter += 1;
        if (counter == 1)
        {
            // Start counting from a second frame (warmup).
            tm.reset();
            tm.start();
        }
    }
    T get()
    {
        std::lock_guard<std::mutex> lock(mutex);
        T entry = this->front();
        this->pop();
        return entry;
    }
    float getFPS()
    {
        tm.stop();
        double fps = counter / tm.getTimeSec();
        tm.start();
        return static_cast<float>(fps);
    }
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex);
        while (!this->empty())
            this->pop();
    }
    unsigned int counter;
private:
    TickMeter tm;
    std::mutex mutex;
};
#endif  // USE_THREADS
int main(int argc, char** argv)
{
    // CommandLineParser parser(argc, argv, keys);
    // const std::string modelName = parser.get<String>("@alias");
    // const std::string zooFile = parser.get<String>("zoo");
    // keys += genPreprocArguments(modelName, zooFile);
    // parser = CommandLineParser(argc, argv, keys);
    // parser.about("Use this script to run object detection deep learning networks using OpenCV.");
    // if (argc == 1 || parser.has("help"))
    // {
    //     parser.printMessage();
    //     return 0;
    // }
    // confThreshold = parser.get<float>("thr");
    // nmsThreshold = parser.get<float>("nms");
    // float scale = parser.get<float>("scale");
    // Scalar mean = parser.get<Scalar>("mean");
    // bool swapRB = parser.get<bool>("rgb");
    // int inpWidth = parser.get<int>("width");
    // int inpHeight = parser.get<int>("height");
    // size_t asyncNumReq = parser.get<int>("async");
    // CV_Assert(parser.has("model"));
    // std::string modelPath = findFile(parser.get<String>("model"));
    // std::string configPath = findFile(parser.get<String>("config"));
    getCoords();
}
inline void preprocess(const Mat& frame, Net& net, Size inpSize, float scale,
                       const Scalar& mean, bool swapRB)
{
    static Mat blob;
    // Create a 4D blob from a frame.
    if (inpSize.width <= 0) inpSize.width = frame.cols;
    if (inpSize.height <= 0) inpSize.height = frame.rows;
    blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false, CV_8U);
    // Run a model.
    net.setInput(blob, "", scale, mean);
    if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
    {
        resize(frame, frame, inpSize);
        Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
        net.setInput(imInfo, "im_info");
    }
}
std::vector<int> postprocess(Mat& frame, const std::vector<Mat>& outs, Net& net, int backend)
{
    static std::vector<int> outLayers = net.getUnconnectedOutLayers();
    static std::string outLayerType = net.getLayer(outLayers[0])->type;
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect> boxes;
    if (outLayerType == "DetectionOutput")
    {
        // Network produces output blob with a shape 1x1xNx7 where N is a number of
        // detections and an every detection is a vector of values
        // [batchId, classId, confidence, left, top, right, bottom]
        CV_Assert(outs.size() > 0);
        for (size_t k = 0; k < outs.size(); k++)
        {
            float* data = (float*)outs[k].data;
            for (size_t i = 0; i < outs[k].total(); i += 7)
            {
                float confidence = data[i + 2];
                if (confidence > confThreshold)
                {
                    int left   = (int)data[i + 3];
                    int top    = (int)data[i + 4];
                    int right  = (int)data[i + 5];
                    int bottom = (int)data[i + 6];
                    int width  = right - left + 1;
                    int height = bottom - top + 1;
                    if (width <= 2 || height <= 2)
                    {
                        left   = (int)(data[i + 3] * frame.cols);
                        top    = (int)(data[i + 4] * frame.rows);
                        right  = (int)(data[i + 5] * frame.cols);
                        bottom = (int)(data[i + 6] * frame.rows);
                        width  = right - left + 1;
                        height = bottom - top + 1;
                    }
                    classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
                    boxes.push_back(Rect(left, top, width, height));
                    confidences.push_back(confidence);
                }
            }
        }
    }
    else if (outLayerType == "Region")
    {
        for (size_t i = 0; i < outs.size(); ++i)
        {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > confThreshold)
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                    // std::cout << classIdPoint.x << '\n';
                }
            }
        }
    }
    else
        CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
    // NMS is used inside Region layer only on DNN_BACKEND_OPENCV for another backends we need NMS in sample
    // or NMS is required if number of outputs > 1
    if (outLayers.size() > 1 || (outLayerType == "Region" && backend != DNN_BACKEND_OPENCV))
    {
        std::map<int, std::vector<size_t> > class2indices;
        for (size_t i = 0; i < classIds.size(); i++)
        {
            if (confidences[i] >= confThreshold)
            {
                class2indices[classIds[i]].push_back(i);
            }
        }
        std::vector<Rect> nmsBoxes;
        std::vector<float> nmsConfidences;
        std::vector<int> nmsClassIds;
        for (std::map<int, std::vector<size_t> >::iterator it = class2indices.begin(); it != class2indices.end(); ++it)
        {
            std::vector<Rect> localBoxes;
            std::vector<float> localConfidences;
            std::vector<size_t> classIndices = it->second;
            for (size_t i = 0; i < classIndices.size(); i++)
            {
                localBoxes.push_back(boxes[classIndices[i]]);
                localConfidences.push_back(confidences[classIndices[i]]);
            }
            std::vector<int> nmsIndices;
            NMSBoxes(localBoxes, localConfidences, confThreshold, nmsThreshold, nmsIndices);
            for (size_t i = 0; i < nmsIndices.size(); i++)
            {
                size_t idx = nmsIndices[i];
                nmsBoxes.push_back(localBoxes[idx]);
                nmsConfidences.push_back(localConfidences[idx]);
                nmsClassIds.push_back(it->first);
            }
        }
        boxes = nmsBoxes;
        classIds = nmsClassIds;
        confidences = nmsConfidences;
        // for(int i =0;i<classIds.size();i++)
        // {
        //     std::cout << confidences[i] << '\n';
        // }
    }
    Rect box = boxes[0];
    std::vector<int> box_val{};
    box_val.push_back(box.x);
    box_val.push_back(box.y);
    box_val.push_back(box.x+box.width);
    box_val.push_back(box.y+box.height);
    return box_val;
    // drawPred(classIds[0], confidences[0], box.x, box.y,box.x + box.width, box.y + box.height, frame);
    // for (size_t idx = 0; idx < boxes.size(); ++idx)
    // {
    //     std::cout << idx << '\n';
    //     Rect box = boxes[idx];
    //     drawPred(classIds[idx], confidences[idx], box.x, box.y,
    //              box.x + box.width, box.y + box.height, frame);
    // }
}
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));
    std::string label = format("%.2f", conf);
    if (!classes.empty())
    {
        // std::cout << classId << ' ' << (int)classes.size() << '\n'; 56 57
        CV_Assert(classId < (int)classes.size());
        label = classes[classId-1] + ": " + label;
    }
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - labelSize.height),
              Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}
void callback(int pos, void*)
{
    confThreshold = pos * 0.01f;
}

// ./a.out --config=detection/darknet/cfg/yolov3.cfg --model=detection/yolov3.weights --classes=object_detection_classes_pascal_voc.txt --width=640 --height=480 --scale=0.00392 --rgb
// g++ custom_obj_det.cpp `pkg-config --cflags --libs opencv4`