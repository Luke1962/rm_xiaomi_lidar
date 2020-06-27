/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************

## req.: data on USB0 (via UART adapter) or gpios 14/15 aka /dev/ttyAMA0.
## build: catkin_make install -DCATKIN_WHITELIST_PACKAGES="xv_11_laser_driver"
## run: rosrun xv_11_laser_driver neato_laser_publisher
 */

 #include <inttypes.h>
//#include <cstdint> // include this header for uint64_t
#include <stdint.h> // include this header for uint64_t
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <xv_11_laser_driver/xv11_laser.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>




//######################################################
//######################################################
// CLASSE Xiaomi_lidar
//######################################################
//######################################################
class Xiaomi_lidar
{
public:
	Xiaomi_lidar(ros::NodeHandle &n);
	

private:

	std::string port;
	int baud_rate;
	std::string frame_id;
	int firmware_number;
	bool print_rpm = false;
	std_msgs::Float64 msg_rps; 
	std_msgs::String msg_speech;
	int  max_distance ;
	ros::Subscriber sub_lidar_enable;

	bool blScanEnable =false;
	void cbk_lidar_enable(const std_msgs::Bool msg);
};

//void cbk_lidar_enable(const std_msgs::Bool msg);
void Xiaomi_lidar::cbk_lidar_enable(const std_msgs::Bool msg){
	
	blScanEnable = msg.data;
}
//----------------------------------------------------


Xiaomi_lidar::Xiaomi_lidar(ros::NodeHandle &nh)
{
 
	//ros::init(argc, argv, "neato_laser_publisher");
	//ros::NodeHandle n;
	//ros::NodeHandle priv_nh("~");

	//-----------------------------------------------------------------
	// Leggo i parametri (launch file)
	//-----------------------------------------------------------------
	nh.param("port", port, std::string("/dev/ttyUSB0"));
	nh.param("baud_rate", baud_rate, 115200);
	nh.param("frame_id", frame_id, std::string("neato_laser"));
	nh.param("firmware_version", firmware_number, 2);
	nh.param("print_rpm", print_rpm, false);
	nh.param("max_distance", max_distance, 5);

	boost::asio::io_service io;
  

	//-----------------------------------------------------------------
	// subscribes
	//-----------------------------------------------------------------
	sub_lidar_enable = nh.subscribe("/lidar_enable", 1, &Xiaomi_lidar::cbk_lidar_enable, this,
							  ros::TransportHints().tcpNoDelay());
	ROS_INFO("subscribed: /lidar_enable");		

	//-----------------------------------------------------------------
	// advertise publishes
	//-----------------------------------------------------------------
    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1000);
    ros::Publisher motor_pub = nh.advertise<std_msgs::Float64>("/lidar_rps",10);
	ros::Publisher pub_speech_once = nh.advertise<std_msgs::String>("/speech_once", 1);

	ros::Time print_time_last = ros::Time::now();


  	try {
    	xv_11_laser_driver::XV11Laser laser( port, baud_rate, firmware_number, io );

	
		laser.max_dist_limit = max_distance;
		////////////////////////////////////////////////////////////////////////////////////
		// main loop
		////////////////////////////////////////////////////////////////////////////////////

		while (ros::ok()) {
			sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
			scan->header.frame_id = frame_id;
			scan->header.stamp = ros::Time::now();
			laser.poll(scan);
			
			// pubblico la velocità di rotazione in rotazioni al sec.
			msg_rps.data=(float)laser.rpms/ 60.0;
			laser_pub.publish(scan);
			motor_pub.publish(msg_rps);


			//ros::Duration updateInterval = time_now -print_time_last;
			ros::Time time_now = ros::Time::now();
			
			
			if (!blScanEnable)
			{
				ROS_INFO_THROTTLE(3,"Waiting for lidar_enable msg");
			}else
			{
				
				if (print_rpm){
					if (time_now > print_time_last + ros::Duration(2) )
					{				
						print_time_last =time_now;		
						
						printf("\nLIDAR rps %f",msg_rps.data);
						
					}				  
				}						
			}

			//Guard su RPS
			if (blScanEnable & (msg_rps.data < 3.5))
			{
				msg_speech.data = "Il Lidar non gira come dovrebbe";
				pub_speech_once.publish(msg_speech);
				ROS_WARN("rotazioni Lidar sotto 3.5Hz : %f",msg_rps.data );
			}
			ros::spinOnce();
		}
		laser.close();
		//return 0;
		
	} catch (boost::system::system_error ex) {
	ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
	//return -1;
	}
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ò
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ò

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xiaomi_lidar_node");

	ros::NodeHandle nh("~");

	Xiaomi_lidar node = Xiaomi_lidar(nh);

	return 0;
}
