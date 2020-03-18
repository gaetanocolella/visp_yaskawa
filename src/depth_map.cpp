#include <visp/vpImage.h> 
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpImageSimulator.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "TooN/TooN.h"
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/vision/vpPose.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <cv_bridge/cv_bridge.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>
#include <bits/stdc++.h> 
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


cv_bridge::CvImagePtr cv_ptr;
int DEPTH_HEIGHT, DEPTH_WIDTH;

void readDepth(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    DEPTH_HEIGHT = msg->height;
    DEPTH_WIDTH = msg->width;
   
    ofstream file;
	 file.open("/home/yaskawa/catkin_visservo/src/add/visp_iiwa/objects/obj.txt");
	 
    for (int i =0; i<DEPTH_WIDTH; i++){
    	for (int j =0; j<DEPTH_HEIGHT; j++){
    		file<< (double)cv_ptr->image.at<short int>(cv::Point(i, j))/1000 <<" ";
    		//cout<<temp<<endl;
    		}
    		file<< " \n";
    }
    file.close();
    depth_sub.shutdown();
    cout<<"finish ----------------------------------------"<<endl;
}      

int main(int argc, char **argv) 
{   
  vpROSRobot robot; 
  robot.setCmdVelTopic("/myrobot/cmd_vel");
  robot.init();

  ros::Rate loop_rate(30);
  ros::NodeHandle n;
  //depth_sub= n.subscribe("/camera/depth/image_rect_raw", 1, readDepth);
  depth_sub= n.subscribe("/camera/aligned_depth_to_color/image_raw", 1, readDepth);
  cmdvel = n.advertise<geometry_msgs::Twist>("/visp_twist", 1);
  current_pose = n.advertise<std_msgs::Float64MultiArray>("/current_pose", 1);
  current_error = n.advertise<std_msgs::Float64MultiArray>("/current_error", 1);
  vpImage<unsigned char> I;

  while (ros::ok()){  
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;

}