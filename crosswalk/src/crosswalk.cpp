#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
#include "std_msgs/Int32.h"
using namespace cv;
using namespace std;
class crosswalk{
	protected:
		Mat Img,hlsImg,BEVhls,BEV,l_images,thresh_Img;
		int x,y,Sum_of_histMax;
		ros::NodeHandle sh;
		std_msgs::Int32 stop_line_data;
		ros::Publisher stop_line_pub;
		ros::Subscriber crosswalk_sub;
	
	public:
		int box_value, thresh_hold_value, rgb_th;
		float cols_offset, row_offset, white_offset_x, white_offset_y;
		int u_xl, u_yl, u_xr, u_yr, d_xl, d_yl, d_xr, d_yr;
		int white_flag1, white_flag2, white_fix1, white_fix2, white_fix3;
		int y_start, y_end, x_start, x_end;
	crosswalk(){
		start();}
	~crosswalk(){
		ROS_INFO("END!");}
	void start(){
		sh.getParam("/crosswalk/box_value",box_value);
		sh.getParam("/crosswalk/cols_offset",cols_offset);
		sh.getParam("/crosswalk/row_offset",row_offset);
		sh.getParam("/crosswalk/thresh_hold_value",thresh_hold_value);
		sh.getParam("/crosswalk/rgb_th",rgb_th);
		sh.getParam("/crosswalk/white_offset_x",white_offset_x);
		sh.getParam("/crosswalk/white_offset_y",white_offset_y);
		sh.getParam("/crosswalk/white_flag1",white_flag1);
		sh.getParam("/crosswalk/white_flag2",white_flag2);
		sh.getParam("/crosswalk/white_fix1",white_fix1);
		sh.getParam("/crosswalk/white_fix2",white_fix2);
		sh.getParam("/crosswalk/white_fix3",white_fix3);

		sh.getParam("/crosswalk/u_xl",u_xl);
		sh.getParam("/crosswalk/u_yl",u_yl);
		sh.getParam("/crosswalk/u_xr",u_xr);
		sh.getParam("/crosswalk/u_yr",u_yr);
		sh.getParam("/crosswalk/d_xl",d_xl);
		sh.getParam("/crosswalk/d_yl",d_yl);
		sh.getParam("/crosswalk/d_xr",d_xr);
		sh.getParam("/crosswalk/d_yr",d_yr);

		
		stop_line_pub = sh.advertise<std_msgs::Int32>("stop_line",10);
		crosswalk_sub=sh.subscribe("/camera1/usb_cam1/image_raw",100, &crosswalk::imageCallback,this);}

	void imageCallback(const sensor_msgs::ImageConstPtr& image) {

        cv_bridge::CvImagePtr cv_ptr;
                try {
                        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
                }
                catch(cv_bridge::Exception& e) {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        return ;
                }
        
        cv::Mat frame = cv_ptr->image;
	Img = frame;
	set_variables_fl();
	processing_IMG();
	set_variables();
	make_small_ROI();
	showing();
	if(Sum_of_histMax > thresh_hold_value) {
                cout<<"stop line is detected"<<endl;
                

                stop_line_data.data = 1;
                stop_line_pub.publish(stop_line_data);
        }

        else {
                stop_line_data.data = 0;
                stop_line_pub.publish(stop_line_data);
        }


        //Crosswalk crosswalk(frame);
        //crosswalk.processing_IMG();
        //crosswalk.set_variables();
        //crosswalk.make_small_ROI();
        //crosswalk.showing();
        //crosswalk.stop_line_detect();
}
void set_variables_fl(){

        x=1280*cols_offset;
        y=960*row_offset;
}
void processing_IMG() {
        resize(Img, Img, Size(1280, 960));

        Point2f src[4];
        src[0] =Point(u_xl, u_yl);//Point(1280*0.4, 960*0.65);
        src[1] =Point(u_xr, u_yr);//Point(1280*0.6, 960*0.65);
        src[2] =Point(d_xl, d_yl);//Point(1280*0.1, 960*0.9);
        src[3] =Point(d_xr, d_yr);//Point(1280*0.9, 900*0.9);
        Point2f dst[4];
        dst[0] = Point(180, 0);
        dst[1] = Point(Img.cols - 180, 0);
        dst[2] = Point(180, Img.rows);
        dst[3] = Point(Img.cols - 180, Img.rows);

        Mat M = getPerspectiveTransform(src, dst);
        warpPerspective(this->Img, BEV, M, BEV.size(), INTER_LINEAR, BORDER_CONSTANT);
		
		vector<Mat> brg_frames(3);
		split(BEV, brg_frames);
		
		
        cvtColor(BEV, hlsImg, COLOR_BGR2HLS);
		//cvtColor(BEV, hlsImg, COLOR_BGR2Lab);
        BEVhls=hlsImg(Rect(0,0,1280,960)).clone();
        vector<Mat> hls_images(3);
        split(BEVhls, hls_images);
        l_images = hls_images[1];
		
		equalizeHist(l_images, l_images);
		y_start = (y*white_offset_y);
         y_end = (y+(l_images.rows/50))/white_offset_y;
         x_start = x*white_offset_x;
         x_end = (l_images.cols - x)/white_offset_x;
         int white_hist[x_end-x_start] = {0,};
          for(int j = 0; j < (x_end-x_start); j++){
              for(int i = 0; i <= (y_end-y_start); i++){
                  white_hist[j] = white_hist[j] + l_images.at<uchar>(i+y_start, j+x_start);
                 }
              white_hist[j] = white_hist[j] / (y_end-y_start+1);
              if(j != 0)
                  white_hist[0] = white_hist[0] + white_hist[j];
          }
         white_hist[0] = white_hist[0] / (x_end - x_start+1);

		 int thresh_value_l;
          if(white_hist[0] < white_flag1)
              thresh_value_l = white_fix1;
          else if(white_hist[0] > white_flag2)
              thresh_value_l = white_fix2;
          else
              thresh_value_l = white_fix3;

         for(int j=0; j<BEVhls.rows; j++){
             for(int i=0; i<BEVhls.cols; i++){
                 if((brg_frames[0].at<uchar>(j, i)<rgb_th)&&(brg_frames[1].at<uchar>(j, i)<rgb_th)    &&(brg_frames[2].at<uchar>(j, i)<rgb_th))
                     l_images.at<uchar>(j, i) = 0;
             }
         }

	
		
//	adaptiveThreshold(l_images, thresh_Img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 1);
			
        threshold(l_images, thresh_Img, 160, 255, THRESH_BINARY);
		
}

void set_variables() {
        x=thresh_Img.cols*cols_offset;
        y=thresh_Img.rows*row_offset;
        Sum_of_histMax = 0;
		
}
void make_small_ROI() {
		double x_ = thresh_Img.cols*cols_offset*2;
		if(((1280-x_)/box_value)<3) {cout<<"Too small box_value"<<endl; return;}
        for(int i=0 ; i<box_value ; i++) {

                        Mat ROI = thresh_Img(Rect(x,y,((1280-x_)/box_value),thresh_Img.rows/50)).clone();
					
                        rectangle(thresh_Img, Point(x, y), Point(x+ROI.cols, y+ROI.rows), Scalar(255, 0, 0), 1, 1, 0);
						
                        int hist[ROI.rows]={0,};

                        for(int k=0;k<ROI.cols;k++) {
                                for(int j=0;j<ROI.rows;j++) {
                                        if(ROI.at<uchar>(j, k)==255){
                                                hist[j]++;
                                        }
                                }
                        }

                        int histMax = 0;
                        int histMax_index = 0;


                        for(int u=0;u<ROI.rows;u++) {
                                if(hist[u] > histMax) {
                                        histMax = hist[u];
                                        histMax_index = u;
                                }
                        }

                        x=x+ROI.cols;

                        if (histMax_index != 0) {
                                 y = y+histMax_index-(ROI.rows/2);;
                        }

                        Sum_of_histMax+=histMax;
        }
}

void showing() {
		line(thresh_Img, Point(x_start, y_start), Point(x_end, y_start), Scalar(255, 0,0), 1);
		line(thresh_Img, Point(x_start, y_end), Point(x_end, y_end), Scalar(255, 255, 255), 1);
		line(thresh_Img, Point(x_start, y_start), Point(x_start, y_end), Scalar(255, 255,255), 1);
		line(thresh_Img, Point(x_end, y_start), Point(x_end, y_end), Scalar(255, 255, 255), 1);
    	imshow("Img", Img);
    	imshow("thresh_Img", thresh_Img);
        waitKey(1);

}
};

int main(int argc, char **argv) {
        ros::init(argc, argv, "crosswalk_node");

        crosswalk cr;

        ros::spin();
        return 0;
}

