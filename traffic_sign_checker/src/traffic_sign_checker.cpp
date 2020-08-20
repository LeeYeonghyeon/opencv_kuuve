#include <iostream>
#include <string>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <traffic_sign_checker/TrafficSign.h>
#include <traffic_sign_checker/TrafficLightJudgment.h>
#include <traffic_sign_checker/TrafficHPA.h>
#include <Direction/direction.h>
using namespace std;
class TrafficSignChecker {
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;


    ros::Publisher traffic_light_judgment_pub_;
	ros::Publisher traffic_sign_pub_;
	ros::Publisher traffic_sign_pub_H;
	ros::Subscriber darknet_sub_;
    ros::Subscriber location_flag_;
    ros::Subscriber stop_line_;
	ros::Subscriber direction_;


    traffic_sign_checker::TrafficLightJudgment traffic_light_judgment_msg_;
	traffic_sign_checker::TrafficSign traffic_sign_msg_;
	traffic_sign_checker::TrafficHPA traffic_HPA_msg_;
    

	// traffic light
	bool is_stop_;
	bool is_go_;
	bool is_gl_;
	bool is_sl_;
	int location_;
	int location;
    //count 
    int count_stop=0;
    int count_go=0;
    int count_gl=0;
    int count_sl=0;
	int count_th;
	int count_tmp;
    //darknet_ros_box
    int xmin=0;
    int xmax=0;
    int ymin=0;
    int ymax=0;

    //stop_line
    int stop_line_data=0;	
    int location_flag=0;
 
	// traffic sign
	bool is_crosswalk_;
	bool is_kidzone_;
	bool is_construction_;
	bool is_parking_;
	bool is_bicycle_;
	bool is_right_;
	bool is_speedbump_;
	bool is_bus_;
	bool is_left_;
	bool is_intersection_;
	
	bool stop_line_data_=false;
	
	
	// handicap sign
	bool is_HPA_;
	double prob_th_;

    // judgment sign
    bool go_=false;
        
  
        
public:
	TrafficSignChecker() {
		initSetup();
	}
	~TrafficSignChecker() {
	}
	
	void initSetup() {
		private_nh_.getParam("/traffic_sign_checker_node/count_th",count_th);
		private_nh_.getParam("/traffic_sign_checker_node/probability_threshold", prob_th_);
        traffic_light_judgment_pub_ = nh_.advertise<traffic_sign_checker::TrafficLightJudgment>("traffic_light_judgment",1);
		traffic_sign_pub_ = nh_.advertise<traffic_sign_checker::TrafficSign>("traffic_sign",1);
		traffic_sign_pub_H = nh_.advertise<traffic_sign_checker::TrafficHPA>("traffic_HPA",1);
	    stop_line_ = nh_.subscribe("stop_line", 10, &TrafficSignChecker::stop_lineCallback, this);
		direction_ = nh_.subscribe("/LR_sign_topic", 1, &TrafficSignChecker::direction_Callback, this);
		darknet_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &TrafficSignChecker::DarknetCallback, this);

	}


	void Judgment() {
        	if (stop_line_data_ == true){
				
				if(location_ == 1){            
					if (is_go_==true)
						go_=true;
					else
						go_=false;}
				else if(location_ == 2){
					if(is_gl_ == true)
						go_=true;
					else
						go_=false;}
				
                }
			
        }



	void stop_lineCallback(const std_msgs::Int32::ConstPtr &stopline_msg){
		int stop_line_data=stopline_msg->data;
	}
	void direction_Callback(const Direction::direction::ConstPtr &direction_msg){
		is_right_ = direction_msg->is_right;
		is_left_ = direction_msg->is_left;
	}


	void DarknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &darknet_msg) {
		int size = darknet_msg->bounding_boxes.size();
                
		for(int i=0; i<size; i++){
		        darknet_ros_msgs::BoundingBox bounding_box = darknet_msg->bounding_boxes[i]; 
		        ROS_INFO("%d BOUNDING BOXES ARE DETECTED.", size);
					
			if(bounding_box.ymax < 360){
				 count_tmp = 1;}
			else if(bounding_box.ymax < 280){
				count_tmp = 0.6;}
			else if(bounding_box.ymax < 200){
				count_tmp = 0.3;}
			else if(bounding_box.ymax < 100){
				count_tmp = 0.1;}
			
                         
	                string class_name = bounding_box.Class; 
		        if(class_name == "Go") {
		                if(bounding_box.probability > prob_th_){
					ROS_INFO("GO TRAFFIC LIGHT DETECTED.");                                     
                                        count_go++;
					ROS_INFO("count : %d",count_go);
                                        
                                       
                                        if(count_go>=count_th * count_tmp){
					
                                            is_go_ =true;
                                            is_stop_=false;
                                            is_gl_=false;
                                            is_sl_=false;  
                                            count_gl=0;
                                            count_sl=0;
                                            count_stop=0;
                                        }
				}
			}
			else if(class_name == "Stop") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("STOP TRAFFIC LIGHT DETECTED.");
                                        
                                        count_stop++;
                                        if(count_stop>=count_th * count_tmp){
                                            is_stop_ =true;
                                            is_go_ =false;
                                            is_gl_=false;
                                            is_sl_=false; 
                                            count_gl=0;
                                            count_sl=0;
                                            count_go=0;
                                        }
				}
			}
			else if(class_name == "GL") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("GL TRAFFIC LIGHT DETECTED.");
					                                        
                                        count_gl++;
                                        if(count_gl>=count_th * count_tmp){
                                            is_gl_ =true;
                                            is_go_ =false;
                                            is_stop_=false;
                                            is_sl_=false; 
                                            count_go=0;
                                            count_sl=0;
                                            count_stop=0;
                                        }
				}
			}
			else if(class_name == "SL") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("SL TRAFFIC LIGHT DETECTED.");
					
                                        count_sl++;
                                        if(count_sl>=count_th * count_tmp){
                                            is_sl_ =true;
                                            is_go_ =false;
                                            is_stop_=false;
                                            is_gl_=false;
                                            count_go=0;
                                            count_gl=0;
                                            count_stop=0;
                                        }
				}
			}
			else if(class_name == "Bus") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Bus traffic sign DETECTED.");
					is_bus_ = true;
				}
			}
			else if(class_name == "Crosswalk") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Crosswalk traffic  sign DETECTED.");
					is_crosswalk_ = true;
				}
			}
			else if(class_name == "Construction") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Construction traffic sign  DETECTED.");
					is_construction_ = true;
				}

			}
          /* 		 else if(class_name == "Right") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("RIGHT TRAFFIC SIGN DETECTED.");
					is_right_ = true;
				}

			}*/
			else if(class_name == "Kidzone") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Kidzone TRAFFIC SIGN DETECTED.");
					is_kidzone_ = true;
				}

			}
			else if(class_name == "Parking") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("PARKING TRAFFIC SIGN DETECTED.");
					is_parking_ = true;
				}

			}
			else if(class_name == "Bicycle") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Bicycle TRAFFIC SIGN DETECTED.");
					is_bicycle_ = true;
				}

			}
			else if(class_name == "Intersection") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Intersection TRAFFIC SIGN DETECTED.");
					is_intersection_ = true;
				}

			}
			else if(class_name == "Speedbump") {
				if(bounding_box.probability > prob_th_) {
					ROS_INFO("Speedbump TRAFFIC SIGN DETECTED.");
					is_speedbump_ = true;
				}
			}
			else if(class_name == "HPA"){
				if(bounding_box.probability > prob_th_){
					ROS_INFO("HPA TRAFFIC SIGN DETECTED.");
					is_HPA_ = true;
				}
			}
			
			else ROS_INFO("THERE IS NO PROPER CANDIDATE.");
		
        if(location_ != location){
			stop_line_data_=false;
			location_ = location;}

        Judgment();


          
        traffic_light_judgment_msg_.go = go_;


		traffic_sign_msg_.is_crosswalk = is_crosswalk_;
		traffic_sign_msg_.is_intersection = is_intersection_;			
		traffic_sign_msg_.is_right = is_right_;	
		traffic_sign_msg_.is_construction = is_construction_;	
		traffic_sign_msg_.is_parking = is_parking_;
		traffic_sign_msg_.is_bicycle = is_bicycle_;
		traffic_sign_msg_.is_left = is_left_;
		traffic_sign_msg_.is_kidzone = is_kidzone_;
		traffic_sign_msg_.is_speedbump = is_speedbump_;
		traffic_sign_msg_.is_bus = is_bus_;
		traffic_HPA_msg_.is_HPA = is_HPA_;
		


                		

			
		traffic_sign_pub_.publish(traffic_sign_msg_);
		traffic_sign_pub_H.publish(traffic_HPA_msg_);
        traffic_light_judgment_pub_.publish(traffic_light_judgment_msg_);
		
		go_ = false;
	
		is_crosswalk_ = false;
		is_intersection_ = false;
		is_left_ = false;
		is_right_ = false;
		is_construction_ = false;
		is_parking_ = false;
		is_bicycle_ = false;
		is_kidzone_ = false;
		is_speedbump_ = false;
		is_bus_ = false;
		is_HPA_ = false;
		
       	        }
        }


};
int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_sign_checker");
	TrafficSignChecker tc;

	ros::spin();

	return 0;
}


