#include "rgb_pose_estimation.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat input = cv_ptr->image.clone();
    input.copyTo(imageCopy);
    cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);
}

void pose_estimation(cv::Mat & intrinsic_parameter, cv::Mat & distortion_coefficient)
{
    //최소 한 개의 id만 포착돼도 pose estimation 가능
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cv::Vec3d rvec, tvec;
        int valid = estimatePoseBoard(corners, ids, board, intrinsic_parameter, distortion_coefficient, rvec, tvec);

        if(valid > 0)
            cv::drawFrameAxes(imageCopy, intrinsic_parameter, distortion_coefficient, rvec, tvec, 3.5);

        std::cout << "Translation: " << tvec[0]
            << "\tRotation: " << rvec[0]
            << std::endl;

    // ----------------x y z yaw pitch roll 계산
	cv::Mat R;
    cv::Rodrigues(rvec,R);
    cv::Mat R_inv = R.inv();

    cv::Mat P = -R_inv*tvec;
    double*p = (double*)P.data;
    printf("x=%1f, y= %1f, z=%1f\n", p[0],p[1],p[2]);

    double unit_z[] = {0,0,1};
    cv::Mat Zc(3,1,CV_64FC1, unit_z);
    cv::Mat Zw = R_inv*Zc;
    double* zw = (double*)Zw.data;

    double pan = atan2(zw[1], zw[0]) - CV_PI/2;

    double tilt = atan2(zw[2], sqrt(zw[0]*zw[0]+zw[1]*zw[1]));

    double unit_x[] = {1,0,0};

    cv::Mat Xc(3, 1, CV_64FC1, unit_x);

    cv::Mat Xw = R_inv*Xc;

    double* xw = (double *)Xw.data;

    double xpan[] = {cos(pan), sin(pan), 0};

    double roll = acos(xw[0]*xpan[0] + xw[1]*xpan[1] + xw[2]*xpan[2]);

    if(xw[2]<0) roll = -roll;

    printf("yaw = %1f, pitch = %1f, roll = %1f",pan,tilt,roll);

    //imshow에 x y z 표시
    vector_to_marker.str(std::string());

    vector_to_marker << std::setprecision(4)<< "x: " << std::setw(8) << p[0];

    vector_to_marker << std::setprecision(4)<< "y: " << std::setw(8) << p[1];

    vector_to_marker << std::setprecision(4)<< "z: " << std::setw(8) << p[2];
    cv::putText(imageCopy, vector_to_marker.str(),
    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 252, 124), 2, CV_AVX);
    cv::imshow("test image", imageCopy);
    cv::waitKey(1);
    }
}