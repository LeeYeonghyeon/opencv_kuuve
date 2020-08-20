#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <sstream>
#include <bitset>
#include <stdint.h>

#define _USE_MATH_DEFINES

using namespace std;

// static const int SYNC_FRAMES = 50;


class PlatformConnector {
private:
	serial::Serial ser_;

	ros::NodeHandle nh_;
	
	ros::Publisher vel_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher odom_pub_;

	ros::Publisher read_pub_;

	ros::Subscriber odom_sub_;
	ros::Subscriber ackermann_sub_;

	geometry_msgs::TwistStamped vel_msg_;
	geometry_msgs::PoseStamped pose_msg_;
	nav_msgs::Odometry odom_msg_;
	
	float wheel_base_;

	unsigned char alive_;
	unsigned char gear_;
	unsigned char speed0_;
	unsigned char speed1_;
	unsigned char steer0_;
	unsigned char steer1_;
	unsigned char front_brake_;
	size_t num_write_;
	unsigned char str_[14];

	bool ackermann_flag_;

public:
	PlatformConnector() {
		initSetup();
	}
	
	~PlatformConnector() {
	}
	void readserial(){
		serialConnect();
		string serial_input;
		unsigned char alive=0x00;
		unsigned char gear = 0x00;
		unsigned char speed_0 = 0x00;
		unsigned char speed_1 = 0x00;
		unsigned char steer_0 = 0x00;
		unsigned char steer_1 = 0x00;
		unsigned char front_break = 0x01;

		unsigned int steer_total = 0;
		unsigned char str[14] = {0x53,0x54,0x58,0x01,0x00,0x00,speed_0,speed_1,steer_0,steer_1,front_break,alive,0x0D,0x0A};
		read_pub_ = nh_.advertise<std_msgs::String>("serial_data", 1000);

		//ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
		//ros::Subscriber sub = nh.subscribe("ackermann", 10, ackermannCallback);


		//ros::Rate loop_rate(5);

		//std::string str;
	

		size_t num_write = 14;

		//size_t num_rev = 18;
		//uint8_t receive[18];	
		//write packet

		if(ser_.available()){
			std_msgs::String result;
			serial_input = ser_.readline(); 

			//cout << "alive:" << (unsigned int)alive << endl;
			//cout << "read:" << serial_input[0] << endl;
			cout << "-------------------START--------------" << endl;
			cout << "gear:" << (int)serial_input[5] << endl;
			cout << "speed0:" << (int)serial_input[6] << endl;
			cout << "speed1:" << (int)serial_input[7] << endl;
			cout << "steer0:" << (int)serial_input[8] << endl;				
			cout << "steer1:" << (int)serial_input[9] << endl;
			bitset<8> brake_test = bitset<8>(serial_input[10]);
			serial_input[10] = (uint8_t)serial_input[10] & 255;
			cout << "brake:" << (unsigned int)serial_input[10] << endl;
			cout << "enc0:" << (int)serial_input[11] << endl;
			cout << "enc1:" << (int)serial_input[12] << endl;
			cout << "enc2:" << (int)serial_input[13] << endl;
			cout << "enc3:" << (int)serial_input[14] << endl;
			cout << "alive:" << (int)serial_input[15] << endl;

			result.data = serial_input[6] + serial_input[7];
			read_pub_.publish(result);
		}
 	}
	void initSetup() {
		//vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("current_velocity", 1);
		pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
		// odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vehicle_state/velocity", 1);
	
		// odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom", 10);
		// course_sub_ = new message_filters::Subscriber<ackermann_msgs::AckermannDriveStamped>(nh_, "course", 1);

		// sync_tp_ = new message_filters::Synchronizer<OdomCourseSync>(OdomCourseSync(SYNC_FRAMES), *odom_sub_, *course_sub_);

		// sync_tp_->registerCallback(boost::bind(&PlatformConnector::OdomCourseCallback, this, _1, _2));
		
	//	odom_sub_ = nh_.subscribe("odom", 10, &PlatformConnector::OdomCallback, this);
		ackermann_sub_ = nh_.subscribe("ctrl_cmd", 10, &PlatformConnector::AckermannCallback, this);

		wheel_base_ = 1.04; //축거
		
		alive_ = 0x00; //0x00: manual mode 
		gear_ = 0x00; // 0x00: foward drive 
		speed0_ = 0x00; // 0~200 actual speed (kph) * 10  
		speed1_ = 0x00;
		steer0_ = 0x00; // -2000~2000 actual steering angle (degree) * 71 +_ 4% negative: left
		steer1_ = 0x00; 
		front_brake_ = 0x01; // 1 no 150 full brake 
		num_write_ = 14;	
		ackermann_flag_ = false;		
	
		ROS_INFO("PLATFORM_CONTROL INITIALIZED");
	}

	/*void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
		pose_msg_.header = odom_msg->header;
		pose_msg_.pose.position = odom_msg->pose.pose.position;

		pose_pub_.publish(pose_msg_);
		ROS_INFO("CURRENT POSE PUBLISHED.");
	}*/

	void AckermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &ackermann_msg) {
		int speed_total = ackermann_msg->drive.speed * 3.6; // to KPM
		int steer_total = ackermann_msg->drive.steering_angle;
		
		if(speed_total < 255 && speed_total > 0){ 
			gear_ = 0x00;
			speed1_ = speed_total * 10;
			speed0_ = 0x00;
			front_brake_ = 0;
		}
		else if(speed_total > -255 && speed_total < 0){
			gear_ = 0x02;
			speed1_ = -speed_total * 10;
			speed0_ = 0x00;
			front_brake_ = 0;
		}
		else if(speed_total == 0){
			speed0_ = 0x00;
			speed1_ = 0x00;
			front_brake_ = 200;
		}
		else{
			gear_ = 0x01;
			speed0_ = 0x00;
			speed1_ = 0x00;
			front_brake_ = 200;
		}

		if(steer_total > 28) steer_total = 28;
		else if(steer_total < -28) steer_total = -28;
		steer_total = steer_total * 71;

		//speed_1 = 0x10; 
		//speed_0 = 0x00, speed_1 = 0x10 > 1.6km/h
		
		steer0_ = steer_total >> 8;
		steer1_ = steer_total & 0xff;

		
		str_[0] = 0x53;
		str_[1] = 0x54;
		str_[2] = 0x58;
		str_[3] = 0x01;	
		str_[4] = 0x00;	
		str_[5] = gear_;
		str_[6] = speed0_;
		str_[7] = speed1_;
		str_[8] = steer0_;
		str_[9] = steer1_;
		str_[10] = front_brake_;
		str_[11] = alive_;	
		str_[12] = 0x0D;	
		str_[13] = 0x0A;	
		
		ackermann_flag_ = true;
	}	

	void serialConnect() {
		try {
			// MRP-2000: /dev/ttyUSB0
			ser_.setPort("/dev/ttyUSB0");
			ser_.setBaudrate(115200);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser_.setTimeout(to);
			ser_.open();
		}
		catch(serial::IOException& e) {
			ROS_ERROR_STREAM("UNABLE TO OPEN SERIAL PORT.");
		}
		if(ser_.isOpen()){}
		else {
			ROS_ERROR_STREAM("UNABLE TO INITIALIZE SERIAL PORT.");
			ros::shutdown();
		}
	}

	void serialProcess() {
		serialConnect();
		if(ser_.available()) {
			string serial_output = ser_.readline();

			unsigned char gear = (int)serial_output[5];
			unsigned char speed_0 = (int)serial_output[6];
			unsigned char speed_1 = (int)serial_output[7];
			unsigned char steer_0 = (int)serial_output[8];
			unsigned char steer_1 = (int)serial_output[9];
		
			float total_steer = (steer_0 + steer_1 * 256) / 71.0;
		
			//vel_msg_.header.stamp = ros::Time::now();	
			//vel_msg_.twist.linear.x = kmph2mps(speed_0 / 10.0);
			//vel_msg_.twist.angular.z = calcAngularVelocity(total_steer, vel_msg_.twist.linear.x);
			
			/*
			odom_msg_.header.stamp = ros::Time::now();
			odom_msg_.header.frame_id = "odom";
		
#
float32 steering_angle          # desired virtual angle (radians)
			
			odom_msg_.twist.twist.angular.z = calcAngularVelocity(total_steer, vel_msg_.twist.linear.x);
			*/
	
			//vel_pub_.publish(vel_msg_);
	
			// odom_pub_.publish(odom_msg_);

			// ROS_INFO("LINEAR_X: %fm/s", vel_msg_.twist.linear.x);
			// ROS_INFO("ANGULAR_Z: %frad/s", vel_msg_.twist.angular.z);
			cout << ackermann_flag_ << endl;
			if(ackermann_flag_){
				ser_.write(str_, num_write_);
				ackermann_flag_ = false;
			}
			
			if(alive_ != 0xff) alive_++;
			else alive_ = 0x00;
		}
	}

	float calcAngularVelocity(float steer, float current_vel) {
		float turn_radius = wheel_base_ / sin(steer * M_PI / 180);
		float angular_velocity = current_vel / turn_radius;
		return angular_velocity; 
	}

	float kmph2mps(float linear_x) {
		return linear_x * 1000 / 3600.0; 
	}

	void run() {
		while(ros::ok()) {
			serialProcess();
			ros::spinOnce();
			ser_.close();
			readserial();
			ros::spinOnce();
			ser_.close();
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "platform_control");
	PlatformConnector pc;
	pc.run();
	
	return 0;
}

