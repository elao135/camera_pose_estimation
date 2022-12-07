#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

//--------------------- 변수 선언 -----------------------------------

cv::Mat image, imageCopy;

std::ostringstream vector_to_marker;
std::vector<int> ids;
std::vector<std::vector<cv::Point2f> > corners;
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7,3.3, 0.7, dictionary);

cv::Mat intrinsic_parameter = (cv::Mat1d(3,3) << 950.3955649641379, 0, 617.4085543989751,
                                             0, 949.664898671666, 384.055444194634,
                                             0,                 0,                 1);
cv::Mat distortion_coefficient = (cv::Mat1d(1,5) << -0.4179201177519848, 0.2459347422574937, 0.000295633578362335, -0.0002136899031111, -0.1050203120225495);

//--------------------- 변수 선언 -----------------------------------//

void pose_estimation(cv::Mat & intrinsic_parameter, cv::Mat & distortion_coefficient);

void imageCallback(const sensor_msgs::ImageConstPtr& msg);