#include "../include/follower.h"
#include <highgui.h>
#include <cv.h>

void SigintHandler(int sig)
{
  ROS_INFO_STREAM("Follower Node Is Shutting Down");
  shutdown();
}

Mat loadFrame(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr tempResult = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat result = tempResult->image.clone();
    return result;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

Mat loadDepthFrame(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr tempResult = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat result = tempResult->image.clone();
    return result;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

Mat createGreyscale(Mat input_image)
{
  Mat result;
  cvtColor(input_image, result, CV_BGR2GRAY);
  return result;
}

Mat createColor(Mat input_image)
{
  Mat result;
  cvtColor(input_image, result, COLOR_GRAY2BGR);
  return result;
}

Mat edgeDetection(Mat input_frame, int lowThreshold, int highThreshold, int kernel_size)
{
  Mat result;
  Canny(input_frame, result, lowThreshold, highThreshold, kernel_size);
  return result;
}

Mat gaussianBlur(Mat input_frame, int size, int point)
{
  Mat result;
  GaussianBlur(input_frame, result, Size(size, size), point, point);
  return result;
}

vector<Vec4i> probLineTransform(Mat input_image, int threshold, int min_line, int max_gap)
{
  vector<Vec4i> result;
  HoughLinesP(input_image, result, 1, CV_PI/180, threshold, min_line, max_gap);
  return result;
}

int longestLineIndex(vector<Vec4i> input_vector)
{
  int result;
  for(size_t i = 0; i < input_vector.size(); i++)
  {
    Vec4i l = input_vector[i];
    int length1 = sqrt(((l[3]-l[1])^2) + ((l[2]-l[0])^2));
    int length2;
    if(i == 0 || length1 >= length2)
    {
      result = i;
      length2 = length1;
    }
  }
  return result;
}

void drawingLines(vector<Vec4i> lines, Mat image, int r, int g, int b)
{
  for(size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    line(image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(b,g,r), 2, LINE_AA);
  }
}

void imageCallbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
  // Load full color frame
  Mat frameRGB = loadFrame(msg);

  // Defining crop area
  Rect myROI(0, 140, 640, 340);

  // Cropping full color image
  Mat croppedFrame = frameRGB(myROI);

  // Apply Gaussian blur
  //GaussianBlur(croppedFrame, croppedFrame, Size(9, 9), 2, 2);

  // Converting cropped image to greyscale
  Mat croppedFrame_gray = createGreyscale(croppedFrame);

  // Edge detection
  Mat edges = edgeDetection(croppedFrame_gray, 25, 100, 3);

  // Converting greyscale image to color for drawing
  Mat pltRGB = createColor(edges);

  // Probabilistic Line Transform
  vector<Vec4i> lines = probLineTransform(edges, 50, 50, 10);

  // Drawing detected lines
  drawingLines(lines, pltRGB, 255, 255, 0);

  // Display video
  imshow("RGB Image", frameRGB);
  imshow("RGB Probabilistic Line Transform", pltRGB);

  waitKey(30);
}

void imageCallbackIR(const sensor_msgs::ImageConstPtr& msg)
{
  // Load full infrared frame
  Mat frameIR = loadFrame(msg);

  // Defining crop area
  Rect myROI(0, 160, 640, 320);

  // Cropping full color image
  Mat croppedFrame = frameIR(myROI);

  // Apply Gaussian blur
  Mat croppedFrameBlurred = gaussianBlur(croppedFrame, 9, 2);

  // Edge detection
  Mat edges = edgeDetection(croppedFrameBlurred, 50, 200, 3);

  // Converting greyscale image to color for drawing
  Mat pltIR = createColor(edges);

  // Probabilistic Line Transform
  vector<Vec4i> lines = probLineTransform(edges, 50, 30, 10);

  // Drawing detected lines
  drawingLines(lines, pltIR, 255, 255, 0);

  imshow("IR Image", frameIR);
  imshow("IR Probabilistic Line Transform", pltIR);

  waitKey(30);
}
