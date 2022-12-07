#include <ros/ros.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <sstream>
#include <string>
#include <stdio.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace std;
using namespace cv;
stringstream ss;

cv::Mat decode;
cv::Mat dst1;

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{

    decode = cv::imdecode(cv::Mat(msg->data), 1);    
    cv::waitKey(1);
}

void Saved_Image(int num_images)
{
    std::stringstream filename;
    filename << "/home/cona/db_ws/src/image_save/save_fold"<< "/" << "image" << num_images << ".jpg";
    //flip(decode,dst1,-1); -> to flip
    cv::imwrite(filename.str(), decode);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_in");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("usb_cam/image_raw/compressed", 100, imageCallback);
    int num_images=0;
    char str[20];
    int CHESSBOARD[2]{7, 10};
    while(ros::ok())
    {
        printf("push spacebar to capture\n");
        scanf(" %[^\n]", &str);
        ros::Rate loop_rate(1);
        loop_rate.sleep();
        ros::spinOnce();
        Saved_Image(num_images++);
        if(num_images>=10)
            break;
        
    }
    std::vector<cv::String> images;

    std::string path = "/home/cona/db_ws/src/image_save/save_fold/image*.jpg";

    cv::glob(path,images);

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> images_points;

    std::vector<cv::Point3f> world_coordinate;

	for (int i = 0; i < CHESSBOARD[1]; i++)		// CHECKERBOARD[1] = 7
	{
		for (int j = 0; j < CHESSBOARD[0]; j++)	// CHECKERBOARD[0] = 10
		{
			world_coordinate.push_back(cv::Point3f(j * 19, i * 19, 0));	// z는 0 이고 (x, y ,z)로 담기니까 (j, i , 0)으로 벡터에 push_back으로 값 담기
			// 실제 값에 대한 정보가 들어가야 하므로 j i 가 아니라 체스보드의 한칸 길이(나의 경우 1.9cm)까지 곱해서 넣어줘어야 함
		}
	}

    cv::Mat frame, gray;
	std::vector<cv::Point2f> corner_points;
	bool success;
	char buf[256];
	int index = 0;

	for (int i=0; i< images.size(); i++)
	{
		frame = cv::imread(images[i]);
		cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
		success = cv::findChessboardCorners(gray, cv::Size(CHESSBOARD[0], CHESSBOARD[1]), corner_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		

		cv::drawChessboardCorners(frame, cv::Size(CHESSBOARD[0], CHESSBOARD[1]), corner_points, success);
		

		object_points.push_back(world_coordinate);
		images_points.push_back(corner_points);

		cv::imshow("Image", frame);

		sprintf(buf, "%d_corner.jpg", index);
		cv::imwrite(buf, frame);

		index++;

		cv::waitKey(0);	

	}

	cv::destroyAllWindows();


	cv::Mat intrinsic_parameter, distortion_coefficient, rotation_matrix, translation_matrix;

	cv::calibrateCamera(object_points, images_points, cv::Size(gray.rows,gray.cols), intrinsic_parameter,distortion_coefficient,rotation_matrix,translation_matrix);

	std::cout << "Intrinsic parameter : " << intrinsic_parameter << "\n\n";
	std::cout << "Distortion coefficient : " << distortion_coefficient<< "\n\n";
	std::cout << "Rotation matrix : " << rotation_matrix << "\n\n";
	std::cout << "Translation matrix : " << translation_matrix << "\n\n";
	std::cout << "world_coordinate : " << world_coordinate << "\n\n";
	std::cout << "corner_points : " << corner_points << "\n\n";

	double fx = intrinsic_parameter.at<double>(0,0);
	double fy = intrinsic_parameter.at<double>(1,1);
	double cx = intrinsic_parameter.at<double>(0,2);
	double cy = intrinsic_parameter.at<double>(1,2);

	double m[] = {fx,0,cx,0,fy,cy,0,0,1};

	cv::Mat undistorted;
	frame = cv::imread(images[0]);
	cv::undistort(frame, undistorted, intrinsic_parameter, distortion_coefficient);
	cv::imshow("Image", undistorted);
	cv::waitKey(0);
	


	cv::Mat camera_matrix(3,3,CV_64FC1,m);
	cv::Mat r, t;


	cv::solvePnP(world_coordinate,corner_points,camera_matrix,distortion_coefficient,r,t);

	cv::Mat R;
	cv::Rodrigues(r,R);
	cv::Mat R_inv = R.inv();

	cv::Mat P = -R_inv*t;
	double*p = (double*)P.data;
	printf("x=%1f, y= %1f, z=%1f", p[0],p[1],p[2]);
}

