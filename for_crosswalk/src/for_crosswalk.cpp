#include <ros/ros.h>
#include<opencv2/opencv.hpp>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<iostream>
#include<sensor_msgs/Image.h>
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "for_crosswalk");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it(nh_);
	image_transport::Publisher pub_a=it.advertise("/camera1/usb_cam1/image_raw",1);
	cv_bridge::CvImagePtr cv_ptr;
	//cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	std::string tmp;
	nh_.getParam("/video1",tmp);
	VideoCapture capture("/home/lee_yeong_hyeon/catkin_ws/src/for_crosswalk/src/2020-08-16-161300.webm");
	
	if(!capture.isOpened()){
		std::cout<<"no!"<<std::endl;
		return 0;
	}
	
	Mat frame;
	sensor_msgs::ImagePtr msg;
	//WImageBuffer3_b image(cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR));
   	//sensor_msgs::ImagePtr msg = sensor_msgs::cv_bridge::cvToImgMsg(image.Ipl(),"bgr8");
	while(ros::ok()){
		capture>>frame;
		
		//msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		 //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		if(frame.empty()){
			std::cout<<"empty"<<std::endl;
			}
		if(waitKey(10)>0)break;
		msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		waitKey(50);	
		pub_a.publish(msg);
		}
//	ros::spin();
}
