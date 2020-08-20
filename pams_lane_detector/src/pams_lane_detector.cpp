#include <iostream>
#include <numeric>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;
using namespace cv;

int fourcc = VideoWriter::fourcc('M','J','P','G');
double fps = 24;
bool isColor = true;
Size size = Size(1280, 720);
VideoWriter writer_L("/home/lee_yeong_hyeon/out_l.avi",fourcc, fps, size, isColor);
VideoWriter writer_R("/home/lee_yeong_hyeon/out_r.avi",fourcc, fps, size, isColor);

class PamsLaneDetector {
protected:
	ros::NodeHandle nh_;
	ros::Subscriber left_image_sub_;
	ros::Subscriber right_image_sub_;
	ros::Publisher ackermann_pub_;

	int mode_select_;

	int l_offset, r_offset, height_set, steer_y;/* set2_lx, set3_lx, set1_rx, set2_rx, set3_rx, set2_ly, set3_ly, set1_ry, set2_ry, set3_ry;*/
	
	
	double mv1;//middle_value
	double mv2;
	double mv3;
	double mv;//real middle_value
	double mva;
	double angle_pid;
	double pid_weight;
	double a1;
	double als;
	double platform_speed; 
	


	double d_error=0.0;
	double p_error=0.0;
	double i_error=0.0;

	VideoCapture left_cap_;
	VideoCapture right_cap_;

	bool is_left_;
	bool is_right_;
	
	int left_hist_value, right_hist_value;
	
	Mat left_frame_;
	Mat right_frame_;
	Mat merge_frame_;
	Mat warped_frame_;
	Mat b_frame_;
	Mat binary_frame_;

	Mat r_frame;
	Mat g_frame;
	Mat b_frame;
	int rgb_th;
	
	double Kp,Kd,Ki;

	Point2f src_[4], dst_[4];

	int left_merge_xpos_, right_merge_xpos_;

	int src_left_upper_xpos_, src_left_upper_ypos_, src_right_upper_xpos_, src_right_upper_ypos_, src_left_lower_xpos_, src_left_lower_ypos_, src_right_lower_xpos_, src_right_lower_ypos_, dst_left_upper_xpos_, dst_left_upper_ypos_, dst_right_upper_xpos_, dst_right_upper_ypos_, dst_left_lower_xpos_, dst_left_lower_ypos_, dst_right_lower_xpos_, dst_right_lower_ypos_;

	int bin_th_;
	int ypix_min_;
	int win_ypix_min_;
	float xpix_dist_;

	int steering_mode_;
	float steering_offset_;

	int cur_left_lane_pos_, cur_right_lane_pos_;
	vector<int> left_lane_xpos_;
	vector<int> right_lane_xpos_;
	vector<int> left_lane_ypos_;
	vector<int> right_lane_ypos_;

	Mat left_coef_;
	Mat right_coef_;

	float prev_steering_angle_;

public:
	PamsLaneDetector() {
		initSetup();	
	}
	~PamsLaneDetector() {
	}
	void initSetup() {
		nh_.getParam("/pams_lane_detector_node/mode_select", mode_select_);
		nh_.getParam("/pams_lane_detector_node/binary_threshold", bin_th_);
		nh_.getParam("/pams_lane_detector_node/xpix_distance", xpix_dist_);
		nh_.getParam("/pams_lane_detector_node/ypix_min", ypix_min_);
		nh_.getParam("/pams_lane_detector_node/win_ypix_min", win_ypix_min_);
		nh_.getParam("/pams_lane_detector_node/steering_offset", steering_offset_);	

		nh_.getParam("/pams_lane_detector_node/left_merge_xpos", left_merge_xpos_);
		nh_.getParam("/pams_lane_detector_node/right_merge_xpos", right_merge_xpos_);
		
		nh_.getParam("/pams_lane_detector_node/l_offset",l_offset);
		nh_.getParam("/pams_lane_detector_node/r_offset",r_offset);
		nh_.getParam("/pams_lane_detector_node/height_set",height_set);
		
		//nh_.getParam("/pams_lane_detector_node/set1_ly",set1_ly);
	   // nh_.getParam("/pams_lane_detector_node/set2_ly",set2_ly);
	   // nh_.getParam("/pams_lane_detector_node/set3_ly",set3_ly);
	   //	nh_.getParam("/pams_lane_detector_node/set1_ry",set1_ry);
		//nh_.getParam("/pams_lane_detector_node/set2_ry",set2_ry);
		//nh_.getParam("/pams_lane_detector_node/set3_ry",set3_ry);
		
		nh_.getParam("/pams_lane_detector_node/steer_y",steer_y);

		nh_.getParam("/pams_lane_detector_node/Kp",Kp);
	    	nh_.getParam("/pams_lane_detector_node/Kd",Kd);
	      	nh_.getParam("/pams_lane_detector_node/Ki",Ki);
		nh_.getParam("/pams_lane_detector_node/als", als);

		nh_.getParam("/pams_lane_detector_node/platform_speed", platform_speed);

		nh_.getParam("/pams_lane_detector_node/rgb_th",rgb_th);

		nh_.getParam("/pams_lane_detector_node/src_left_upper_xpos", src_left_upper_xpos_);
		nh_.getParam("/pams_lane_detector_node/src_left_upper_ypos", src_left_upper_ypos_);
		nh_.getParam("/pams_lane_detector_node/src_right_upper_xpos", src_right_upper_xpos_);
		nh_.getParam("/pams_lane_detector_node/src_right_upper_ypos", src_right_upper_ypos_);
		nh_.getParam("/pams_lane_detector_node/src_left_lower_xpos", src_left_lower_xpos_);
		nh_.getParam("/pams_lane_detector_node/src_left_lower_ypos", src_left_lower_ypos_);
		nh_.getParam("/pams_lane_detector_node/src_right_lower_xpos", src_right_lower_xpos_);
		nh_.getParam("/pams_lane_detector_node/src_right_lower_ypos", src_right_lower_ypos_);	
		
		nh_.getParam("/pams_lane_detector_node/dst_left_upper_xpos", dst_left_upper_xpos_);	
		nh_.getParam("/pams_lane_detector_node/dst_left_upper_ypos", dst_left_upper_ypos_);	
		nh_.getParam("/pams_lane_detector_node/dst_right_upper_xpos", dst_right_upper_xpos_);	
		nh_.getParam("/pams_lane_detector_node/dst_right_upper_ypos", dst_right_upper_ypos_);	
		nh_.getParam("/pams_lane_detector_node/dst_left_lower_xpos", dst_left_lower_xpos_);	
		nh_.getParam("/pams_lane_detector_node/dst_left_lower_ypos", dst_left_lower_ypos_);	
		nh_.getParam("/pams_lane_detector_node/dst_right_lower_xpos", dst_right_lower_xpos_);	
		nh_.getParam("/pams_lane_detector_node/dst_right_lower_ypos", dst_right_lower_ypos_);

		nh_.getParam("/pams_lane_detector_node/pid_weight", pid_weight);	
		
		src_[0].x = src_left_upper_xpos_;
		src_[0].y = src_left_upper_ypos_;
		src_[1].x = src_right_upper_xpos_;
		src_[1].y = src_right_upper_ypos_;
		src_[2].x = src_left_lower_xpos_;
		src_[2].y = src_left_lower_ypos_;
		src_[3].x = src_right_lower_xpos_;
		src_[3].y = src_right_lower_ypos_;

		dst_[0].x = dst_left_upper_xpos_;
		dst_[0].y = dst_left_upper_ypos_;
		dst_[1].x = dst_right_upper_xpos_;
		dst_[1].y = dst_right_upper_ypos_;
		dst_[2].x = dst_left_lower_xpos_;
		dst_[2].y = dst_left_lower_ypos_;
		dst_[3].x = dst_right_lower_xpos_;
		dst_[3].y = dst_right_lower_ypos_;
		
		int height_set_;
		is_left_ = false;
		is_right_ = false;
		if(720%height_set!=0) {
			while(1)
				ROS_INFO("height Parameter is wrong!!");
				
		}
		height_set_ = (720/height_set) +1;
			
		// 0: ONLY RIGHT LANE
		// 1: ONLY LEFT LANE
		// 2: BOTH LANE
		// 3: NO LANE
		steering_mode_ = 0;

			left_image_sub_ = nh_.subscribe("/camera1/usb_cam1/image_raw", 10, &PamsLaneDetector::leftImageCallback, this);
			right_image_sub_ = nh_.subscribe("/camera2/usb_cam2/image_raw", 10, &PamsLaneDetector::rightImageCallback, this);
			ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);	

		left_coef_ = Mat::zeros(3, 1, CV_32F);
		right_coef_ = Mat::zeros(3, 1, CV_32F);

		prev_steering_angle_ = 0.0;
	}
	int getMode() {
		return mode_select_;
	}
	void leftImageCallback(const sensor_msgs::Image::ConstPtr &left_image) {
		cv_bridge::CvImagePtr cv_ptr;
		
		try {
			cv_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e) {
			ROS_ERROR("CV_BRIDGE EXCEPTION: %s", e.what());
			return;
		}

		left_frame_ = cv_ptr->image;

		resize(left_frame_, left_frame_, Size(1280, 720));

		is_left_ = true;
	}
	void rightImageCallback(const sensor_msgs::Image::ConstPtr &right_image) {
		cv_bridge::CvImagePtr cv_ptr;
		
		try {
			cv_ptr = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e) {
			ROS_ERROR("CV_BRIDGE EXCEPTION: %s", e.what());
			return;
		}

		right_frame_ = cv_ptr->image;

		resize(right_frame_, right_frame_, Size(1280, 720));

		is_right_ = true;
	}

	void warp() {
		Mat M = getPerspectiveTransform(src_, dst_);
		Mat warped(merge_frame_.rows, merge_frame_.cols, merge_frame_.type());
		warpPerspective(merge_frame_, warped_frame_, M, warped_frame_.size(), INTER_LINEAR, BORDER_CONSTANT);

		line(merge_frame_, src_[0], src_[1], Scalar(0, 0, 255), 2);
		line(merge_frame_, src_[1], src_[3], Scalar(0, 0, 255), 2);
		line(merge_frame_, src_[3], src_[2], Scalar(0, 0, 255), 2);
		line(merge_frame_, src_[2], src_[0], Scalar(0, 0, 255), 2);
	}
	void merge() {
		Mat left_roi = left_frame_(Range(0, 720), Range(0, left_merge_xpos_));
		Mat right_roi = right_frame_(Range(0, 720), Range(right_merge_xpos_, 1280));

		hconcat(left_roi, right_roi, merge_frame_);
	}
	void binary() {
		Mat lab_frame_;
	
//		vector<Mat> lab_frames(3);
		vector<Mat> brg_frames(3);
		
		split(warped_frame_,brg_frames);
		b_frame=brg_frames[0];
		r_frame=brg_frames[1];
		g_frame=brg_frames[2];		

//		cvtColor(warped_frame_, lab_frame_, CV_BGR2Lab);
		cvtColor(warped_frame_, lab_frame_, CV_BGR2GRAY);
//		split(lab_frame_, lab_frames);

//		b_frame_ = lab_frames[2];
		b_frame_ = lab_frame_;

		for(int j=0; j<warped_frame_.rows; j++){
			for(int i=0; i<warped_frame_.cols;i++){
				if( (b_frame.at<uchar>(j, i) <rgb_th)&&(r_frame.at<uchar>(j, i) <rgb_th)&&(g_frame.at<uchar>(j, i) <rgb_th) ){
					b_frame_ .at<uchar>(j, i)=0;				
				}						
			}		
		}
		
//		lab_frames.clear();
		brg_frames.clear();

		GaussianBlur(b_frame_, b_frame_, Size(0, 0), 1);

		threshold(b_frame_, binary_frame_, bin_th_, 255, THRESH_BINARY); 
	}
	void hist() {
		int center_pos = binary_frame_.cols / 2;
		int left_hist[center_pos] = {0, };
		int right_hist[center_pos] = {0, };

		int left_lane_pos = -1;
		int right_lane_pos = -1;

		for(int j=0;j<binary_frame_.rows;j++) {
			for(int i=0;i<binary_frame_.cols;i++) {
				if(binary_frame_.at<uchar>(j, i) == 255) {
					if(i < center_pos) left_hist[i]++;
					else right_hist[i - center_pos]++;
				}
			}
		}
		
		int left_max = 0;
		int right_max = 0;

		for(int i=0;i<center_pos;i++) {
			if((left_hist[i] > left_max) && (left_hist[i] > ypix_min_)) {
				left_max = left_hist[i];
				left_lane_pos = i;
			}
			if((right_hist[i] > right_max) && (right_hist[i] > ypix_min_)) {
				right_max = left_hist[i];
				right_lane_pos = i + center_pos;
			}
		}

		cur_left_lane_pos_ = left_lane_pos;
		cur_right_lane_pos_ = right_lane_pos;	

		// ROS_INFO("LEFT LANE POS: %d", cur_left_lane_pos_);
		// ROS_INFO("RIGHT LANE POS: %d", cur_right_lane_pos_);
		
	}
	void window() {
		ackermann_msgs::AckermannDriveStamped ackermann_msg;

		int set1_ly[steer_y+1] = {-1, 0};
		int set1_lx[steer_y+1] = {-1, 0};
		int set1_rx[steer_y+1] = {-1, 0};
		int set1_ry[steer_y+1] = {-1, 0};

		if(cur_left_lane_pos_ == -1 && cur_right_lane_pos_ != -1) {
			ROS_INFO("CALCULATE STEERING ANGLE BY RIGHT LANE."); 
			steering_mode_ = 0;
		}
		else if(cur_left_lane_pos_ != -1 && cur_right_lane_pos_ == -1) {
			ROS_INFO("CALCULATE STEERING ANGLE BY LEFT LANE."); 
			steering_mode_ = 1;
		}
		else if(cur_left_lane_pos_ != -1 && cur_right_lane_pos_ != -1) {
			ROS_INFO("CALCULATE STEERING ANGLE BY BOTH LANE.");
			steering_mode_ = 2;
		}
		else {
			ROS_INFO("THERE IS NO LANE TO REFER.");
			steering_mode_ = 3;
		}

		int left_center_point = cur_left_lane_pos_;
		int right_center_point = cur_right_lane_pos_;
		
	
		int sethist_1 = l_offset*2;
        int sethist_2 = r_offset*2;
		
			for(int k=1;k<(720/height_set)+1;k++) {
				

				int left_hist[sethist_1] = {0, };
				int right_hist[sethist_2] = {0, };

				vector<int> left_hist_xpos;
				vector<int> right_hist_xpos;
				vector<int> left_hist_ypos;
				vector<int> right_hist_ypos;

				int left_offset = left_center_point - l_offset;
				
				int right_offset = right_center_point - r_offset;			
				
				if(left_center_point <= l_offset) left_offset = 0;

				
				for(int j=binary_frame_.rows-height_set*k;j<=binary_frame_.rows-height_set*(k-1);j++) {
					int start_idx;
					int end_idx;			
					
					if(steering_mode_ != 0) {
						if(left_center_point >= l_offset) start_idx = left_center_point - l_offset;
						else start_idx = 0;
						
						for(int i=start_idx;i<=left_center_point+l_offset;i++) {
							if(binary_frame_.at<uchar>(j, i) == 255) {
								left_hist[i - left_offset]++;
								left_hist_xpos.push_back(i);
								left_hist_ypos.push_back(j);
							}
						}
					}
					if(steering_mode_ != 1) {
						if(right_center_point <= (1280-r_offset)) end_idx = right_center_point + r_offset;
						else end_idx = 1280;
						for(int i=right_center_point-r_offset;i<=end_idx;i++) {
							if(binary_frame_.at<uchar>(j, i) == 255) {
								right_hist[i - right_offset]++;
								right_hist_xpos.push_back(i);
								right_hist_ypos.push_back(j);
							}
						}
					}
					

				}
			

				int left_max = 0;
				int right_max = 0;
				int left_pos = -1;
				int right_pos = -1;

				for(int i=0;i<l_offset*2;i++) {
					if(steering_mode_ != 0) {
						if(left_hist[i] > left_max && left_hist[i] > win_ypix_min_) {
							left_max = left_hist[i];
							left_pos = i + left_offset; 
						}
					}}
				for(int i=0;i<r_offset*2;i++){
					if(steering_mode_ != 1) {	
						if(right_hist[i] > right_max && right_hist[i] > win_ypix_min_) {
							right_max = right_hist[i];
							right_pos = i + right_offset;				
						}
					}
				}
				

				if(steering_mode_ != 0) {
					left_center_point = left_pos;
					left_lane_xpos_.insert(left_lane_xpos_.end(), left_hist_xpos.begin(), left_hist_xpos.end());
					left_lane_ypos_.insert(left_lane_ypos_.end(), left_hist_ypos.begin(), left_hist_ypos.end());
				}
				if(steering_mode_ != 1) {
					right_center_point = right_pos;
					right_lane_xpos_.insert(right_lane_xpos_.end(), right_hist_xpos.begin(), right_hist_xpos.end());
					right_lane_ypos_.insert(right_lane_ypos_.end(), right_hist_ypos.begin(), right_hist_ypos.end());
				}


				left_hist_xpos.clear();
				right_hist_xpos.clear();
				left_hist_ypos.clear();
				right_hist_ypos.clear();
			}
		
		
		
		if(steering_mode_ != 3 && steering_mode_ != 0) {
			left_coef_ = polyfit(left_lane_ypos_, left_lane_xpos_);
			 	
			for(int i =0; i<binary_frame_.rows;i++){
                                int cols = left_coef_.at<float>(2,0)*pow(i,2) + left_coef_.at<float>(1,0)*i + left_coef_.at<float>(0,0);
				if(cols < 640){
                                	line(warped_frame_,Point(cols, i), Point(cols, i) , Scalar(0, 0, 255), 10);
				
					for(int z=0;z<=steer_y;z++){
						if(i == z*(720/steer_y)){
							set1_ly[z]=i;
							set1_lx[z]=cols;
						}
					}
						
				

			}
		}
		}	
		if(steering_mode_ != 3 && steering_mode_ != 1) {
			right_coef_ = polyfit(right_lane_ypos_, right_lane_xpos_);
			
			for(int i =0; i<binary_frame_.rows;i++){
                                int cols = right_coef_.at<float>(2,0)*pow(i,2) + right_coef_.at<float>(1,0)*i + right_coef_.at<float>(0,0);
				if(cols > 640){
                                	line(warped_frame_,Point(cols, i), Point(cols, i) , Scalar(0, 0, 255), 10);					
				 	
					for(int z=0; z<=steer_y; z++){
						if(i == z*(720/steer_y)){
							set1_ry[z] = i;
							set1_rx[z] = cols;
                        	}				
                        }	
		}
	}	

}		
		int count = 0;
		double mv2 = 0;
		double atan_angle = 0;
		double atan_angler =0;
		double atan_anglel =0;
		
		// CALCULATE STEERING ANGLE BY ONLY RIGHT LANE		
		if(steering_mode_ == 0 && steering_mode_ != 1) {
						
			atan_angle=-atan(right_coef_.at<float>(1,0))*180/M_PI;
			if (atan_angle<90)	ackermann_msg.drive.steering_angle = atan_angle ;
			else ackermann_msg.drive.steering_angle = ( 90- atan_angle) ;
			ROS_INFO("Right lane angle: %f",atan_angle);
		}

		// CALCULATE STEERING ANGLE BY ONLY LEFT LANE
		else if(steering_mode_ != 0 && steering_mode_ == 1) {
		
			atan_angle=-atan(left_coef_.at<float>(1,0))*180/M_PI;
			if (atan_angle<90)	ackermann_msg.drive.steering_angle = atan_angle ;
			else ackermann_msg.drive.steering_angle = ( 90- atan_angle) ;
			ROS_INFO("Left lane angle: %f", atan_angle);			
		}
		// CALCULATE STEERING ANGLE BY BOTH LANE
		else if(steering_mode_ == 2) {
			
			for(int j=0; j <= steer_y; j++){
				if(set1_ly[j] == -1){
					 set1_ly[j] = 0;
					 set1_lx[j] = 0;
				}
				if(set1_ry[j] == -1){
					set1_ry[j] = 0;
					set1_rx[j] = 0;
				}
				
				mv1 = (set1_rx[j]+set1_lx[j])/2;
				if(mv1 != 0){
					count += 1;
					mv2 += mv1;
			}
		}
		mv2 = mv2/count;
		if(mv2 == 0)
			mv2 = 640;
			
		atan_anglel=-atan(left_coef_.at<float>(1,0))*180/M_PI;
		atan_angler=-atan(right_coef_.at<float>(1,0))*180/M_PI;

		if (atan_anglel>=90)	atan_anglel = (90- atan_anglel);
		if (atan_angler>=90)	atan_angler = (90- atan_anglel);
		
		atan_angle = (atan_angler +atan_anglel)/2;

		mva = -(640-mv2)*pid_weight * als  + (atan_angle);		
		angle_pid = PID(-mva);
		angle_pid = mva;
			
			ackermann_msg.drive.steering_angle =angle_pid;
			ROS_INFO("Both lane angle: %f",angle_pid);
		}		
		
		else ackermann_msg.drive.steering_angle = prev_steering_angle_;
				
		if (ackermann_msg.drive.steering_angle>20 || ackermann_msg.drive.steering_angle<-20){
			ackermann_msg.drive.speed = platform_speed;
			ROS_INFO("speed: %f",ackermann_msg.drive.speed);
		}
		else {
			ackermann_msg.drive.speed = platform_speed;
			ROS_INFO("speed: %f",ackermann_msg.drive.speed);
		}



		if(( (prev_steering_angle_ - ackermann_msg.drive.steering_angle)>15) || ( (prev_steering_angle_ - ackermann_msg.drive.steering_angle)<-15) ){
			ackermann_msg.drive.steering_angle = prev_steering_angle_;
		}

		ackermann_pub_.publish(ackermann_msg);
		prev_steering_angle_ = ackermann_msg.drive.steering_angle;


		left_lane_xpos_.clear();
		right_lane_xpos_.clear();
		left_lane_ypos_.clear();
		right_lane_ypos_.clear();
	}
	void run() {
		if(is_left_ && is_right_) {	
			merge();
			warp();
			binary();
			hist();
			window();
			// imshow("LEFT FRAME", left_frame_);
			// imshow("RIGHT FRAME", right_frame_);
			writer_L.write(left_frame_);
			writer_R.write(right_frame_);
			imshow("MERGE FRAME", merge_frame_);
			imshow("WARPED FRAME", warped_frame_);
			imshow("BINARY FRAME", binary_frame_);	
			
			is_left_ = false;
			is_right_ = false;

			waitKey(1000 / 24.0);
		}
		// else ROS_INFO("WAIT FOR IMAGES.");
	}
	double gaussian(double x, double mu, double sig) {
		return exp((-1)*pow(x - mu, 2.0) / (2 * pow(sig, 2.0)));
	}
	Mat polyfit(vector<int> xpos, vector<int> ypos) {
		Mat coef(3, 1, CV_32F);
		int i,j,k;
		int N = xpos.size();
		int n = 2;
		float x[N], y[N];
		for(int q=0;q<N;q++) {
			x[q] = xpos[q];
			y[q] = ypos[q];
		}
		float X[2*n+1];                        
	    	for (i=0;i<2*n+1;i++)
	    	{
			X[i]=0;
			for (j=0;j<N;j++)
		    		X[i]=X[i]+pow(x[j],i);        
	    	}
	    	float B[n+1][n+2],a[n+1];            
	    	for (i=0;i<=n;i++)
			for (j=0;j<=n;j++)
		    		B[i][j]=X[i+j];           
	    	float Y[n+1];                    
	    	for (i=0;i<n+1;i++)
	    	{    
			Y[i]=0;
			for (j=0;j<N;j++)
			Y[i]=Y[i]+pow(x[j],i)*y[j];        
	    	}
	    	for (i=0;i<=n;i++)
			B[i][n+1]=Y[i];                
	    	n=n+1;               
	    	for (i=0;i<n;i++)                    
			for (k=i+1;k<n;k++)
		    		if (B[i][i]<B[k][i])
		        		for (j=0;j<=n;j++)
		        		{
		            			float temp=B[i][j];
		            			B[i][j]=B[k][j];
		           			B[k][j]=temp;
		       			 }	
	    
	    	for (i=0;i<n-1;i++)           
			for (k=i+1;k<n;k++)
		   	{
		       		 float t=B[k][i]/B[i][i];
		      		 for (j=0;j<=n;j++)
		           		 B[k][j]=B[k][j]-t*B[i][j];    
		    	}
	    	for (i=n-1;i>=0;i--)               
	    	{                        
			a[i]=B[i][n];                
			for (j=0;j<n;j++)
		    		if (j!=i)
		        		a[i]=a[i]-B[i][j]*a[j];
			a[i]=a[i]/B[i][i];
			coef.at<float>(i, 0) = a[i];
	    	}
		return coef;
	}

	double PID(double error){

		d_error = error -p_error;
		p_error=error;
		i_error+=error;
		return Kp*p_error+Ki*i_error+Kd*d_error; 	
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "pams_lane_detector");
	PamsLaneDetector pld;
	
	while(ros::ok()) {
		pld.run();
		ros::spinOnce();
		
	}

	return 0;
}
