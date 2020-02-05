#ifndef follower_h
#define follower_h

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <vector>
#include <signal.h>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <librealsense2/rs.hpp>

using namespace std;
using namespace cv;
using namespace ros;

void SigintHandler(int sig);
Mat loadFrame(const sensor_msgs::ImageConstPtr& msg);
Mat loadDepthFrame(const sensor_msgs::ImageConstPtr& msg);
Mat createGreyscale(Mat input_image);
Mat createColor(Mat input_image);
Mat edgeDetection(Mat input_frame, int lowThreshold, int highThreshold, int kernel_size);
Mat gaussianBlur(Mat input_frame, int size, int point);
vector<Vec4i> probLineTransform(Mat input_image, int threshold, int min_line, int max_gap);
int longestLineIndex(vector<Vec4i> input_vector);
void drawingLines(vector<Vec4i> lines, Mat image, int r, int g, int b);
void imageCallbackRGB(const sensor_msgs::ImageConstPtr& msg);
void imageCallbackIR(const sensor_msgs::ImageConstPtr& msg);

#endif
