#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <Direction/direction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace std;

class Direction_cl
{	protected:
	ros::NodeHandle nh;
	

	ros::Publisher LR_sign_pub_;
	ros::Subscriber darknet_sub_;
	ros::Subscriber usb_cam_sub_;
	Direction::direction direction_msg_;
	public:
	Mat image_frame;

	Direction_cl(){
	LR_sign_pub_ = nh.advertise<Direction::direction>("/LR_sign_topic", 1);
	usb_cam_sub_ = nh.subscribe("/camera1/usb_cam1/image_raw", 100, &Direction_cl::usb_cam_callback, this); 
	darknet_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 10, &Direction_cl::DirectionCallback,this);
	}
	~Direction_cl(){}
	
	void usb_cam_callback(const sensor_msgs::ImageConstPtr& image){
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		
		image_frame = cv_ptr->image;
	}
	void DirectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &darknet_msg)
	{	
		int size = darknet_msg ->bounding_boxes.size();
		
		for(int i=0;i<size;i++)
		{	bool is_right_ = false;
			bool is_left_ = false;
			darknet_ros_msgs::BoundingBox bounding_box = darknet_msg->bounding_boxes[i];

			int y_min = bounding_box.ymin;
			int y_max = bounding_box.ymax;
			int x_min = bounding_box.xmin;
			int x_max = bounding_box.xmax;

			string class_name = bounding_box.Class;
			if(class_name == "Left" || class_name == "Right"){
				if(bounding_box.probability > 0.8) //LR thresh hold
			{	
				int range_count = 0;
	
				Scalar red(204, 129, 1);
				Scalar blue(255, 0, 0);
				Scalar yellow(0, 255, 255);
				Scalar magenta(255, 0, 255);

				Mat rgb_color = Mat(1, 1, CV_8UC3, red);
				Mat hsv_color;
			
			cvtColor(rgb_color, hsv_color, COLOR_BGR2HSV);
	
			int hue = (int)hsv_color.at<Vec3b>(0, 0)[0];
			int saturation = (int)hsv_color.at<Vec3b>(0, 0)[1];
			int value = (int)hsv_color.at<Vec3b>(0, 0)[2];

			//cout << "hue = " << hue << endl;
			//cout << "saturation = " << saturation << endl;
			//cout << "value = " << value << endl;

			int low_hue = hue - 10;
			int high_hue = hue + 10;

			int low_hue1 = 0, low_hue2 = 0;
			int high_hue1 = 0, high_hue2 = 0;

			if (low_hue < 10 ) 
			{
				range_count = 2;
				high_hue1 = 180;
				low_hue1 = low_hue + 180;
				high_hue2 = high_hue;
				low_hue2 = 0;
			}
			else if (high_hue > 170) 
			{
				range_count = 2;
				high_hue1 = low_hue;
				low_hue1 = 180;
				high_hue2 = high_hue - 180;
				low_hue2 = 0;
			}
			else 
			{
				range_count = 1;
				low_hue1 = low_hue;
				high_hue1 = high_hue;
			}
	
			Mat img_frame,img_hsv;

			/*img_frame = imread("/home/jungmin/Desktop/left.jpg");
			if(img_frame.empty()){
				cout << "ERROR" << endl;
				exit(1);
			}*/
			
			cvtColor(img_frame, img_hsv, COLOR_BGR2HSV);

			Mat img_mask1, img_mask2;
	
			inRange(img_hsv, Scalar(low_hue1, 50, 50), Scalar(high_hue1, 255, 255), img_mask1);
			if (range_count == 2) {
				inRange(img_hsv, Scalar(low_hue2, 50, 50), Scalar(high_hue2, 255, 255), img_mask2);
				img_mask1 |= img_mask2;
			}
	
			Mat ROI1;
			Rect rect(x_min,y_min,(x_max-x_min),(y_max-y_min));
			ROI1 = img_mask1(rect);

			//morphological opening 작은 점들을 제거 
			erode(ROI1, ROI1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(ROI1, ROI1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			//morphological closing 영역의 구멍 메우기 
			dilate(ROI1, ROI1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(ROI1, ROI1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


			//라벨링 
			Mat img_labels, stats, centroids;
			int numOfLables = connectedComponentsWithStats(ROI1, img_labels,
				stats, centroids, 8, CV_32S);
	
			//영역박스 그리기
			int max = -1, idx = 0;
			for (int j = 1; j < numOfLables; j++) {
				int area = stats.at<int>(j, CC_STAT_AREA);
				if (max < area)
				{
					max = area;
					idx = j;
				}
			}
		

			int left = stats.at<int>(idx, CC_STAT_LEFT);
			int top = stats.at<int>(idx, CC_STAT_TOP);
			int width = stats.at<int>(idx, CC_STAT_WIDTH);
			int height = stats.at<int>(idx, CC_STAT_HEIGHT);

			//이미지 크롭 및 리사이즈
			Mat ROI2, ROI_resize;
			Rect rect_(left,top,width,height);
			ROI2 = ROI1(rect_);

			resize(ROI2, ROI_resize,Size(1028,1028),0,0,CV_INTER_NN);
	
			int right_pxl=0; 
			int left_pxl=0;
			int get_right_pxl=0;
			int get_left_pxl=0;
		/*	int histsum_left = 0;
			int histsum_right = 0;



			int hist[ROI_resize.cols]={0,};
			int hist_result[ROI_resize.cols
			for (int k=0;k<(ROI_resize.cols/2);k++)
			{
				for(int j=0;j<100;j++)
				{
						if(ROI_resize.at<uchar>(j,k) == 0)
						{
							histsum_left++;
						}
				}
			}

			for (int k=(ROI_resize.cols/2);k<ROI_resize.cols;k++)
			{
				for(int j=0; j<100;j++)
				{
					if(ROI_resize.at<uchar>(j,k) == 0)
					{
						histsum_right++;
					}
				}
			}
		
			if(histsum_left < histsum_right)
			{
				cout << "right_sign" << endl;
			}

			if(histsum_left >  histsum_right)
			{
				cout << "left_sign" << endl;
			}
			
			if(histsum_left == histsum_right)
			{
				cout << "error_sign" << endl;
			}
		*/
			int right_count = 0;
			int left_count = 0;

			for(int k=250; k<400 ; k+=10)
			{
				for(int i=0;i<512;i++){
					if (ROI_resize.at<uchar>(k,512+i) >= 255) goto skip00;
					get_right_pxl = i;
				}

				skip00: right_pxl = get_right_pxl; 
				cout << "right_pxl: " << get_right_pxl << endl;

				for(int i=0;i<512;i++){
					if (ROI_resize.at<uchar>(k,512-i) >= 255) goto skip01;
					get_left_pxl = i;
				}
				skip01: left_pxl = get_left_pxl;
				cout << "left_pxl: " << get_left_pxl  << endl;

					if(get_right_pxl > get_left_pxl)
					{
						right_count ++;
						cout << "right_count: " << right_count << endl;
					}
			
					if(get_right_pxl < get_left_pxl)
					{
						left_count++;
						cout << "left_count: "<< left_count << endl;
					}
			
					if(get_right_pxl == get_left_pxl)
					{
						cout << "same count " << endl;
					}	
				if(right_count > left_count)
				{
					cout << "right_sign" << endl;
				}
		
				if(right_count < left_count)
				{
					cout << "left_sign" << endl;
				}
	
				if(right_count == left_count)
				{
					cout << "error" << endl;
				}
			}
		/*	imshow("crop_img", ROI_resize);
			imshow("이진화 영상", img_mask1);
			imshow("원본 영상", img_frame);
		*/	
			if(right_count > left_count)
			{
				is_right_ = true;
			}

			if(right_count < left_count)
			{
				is_left_ = true;
			}

			direction_msg_.is_right = is_right_;
			direction_msg_.is_left = is_left_;
			
			LR_sign_pub_.publish(direction_msg_);

		//	waitKey(0);
		}
	}
	}
}
};


int main(int argc, char **argv)
{	ros::init(argc, argv, "LR_sign_node");

	Direction_cl dr;

	ros::spin();

	return 0;
}
