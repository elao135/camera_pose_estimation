#include "rgb_pose_estimation.cpp"


int main(int argc, char ** argv)
{
    ros::init(argc,argv,"image_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw/", 1, imageCallback);
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        pose_estimation(intrinsic_parameter,distortion_coefficient);
    }

    return 0;
}