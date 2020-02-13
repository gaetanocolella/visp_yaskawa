#include <ros/ros.h>
#include "TooN/TooN.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include "sensor_msgs/JointState.h"
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "Traj_Generators/Line_Segment_Traj.h"
#include "Robots/MotomanSIA5F.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "Traj_Generators/Vector_Independent_Traj.h"
#include <std_msgs/Bool.h>
#include "visp_common/ChooseObject.h"
#include <fstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace TooN;

#define Z_OBJ 0.30
#define tf 20.0
#define NUM_JOINTS 7

std_msgs::Bool stop_traj;
Vector<NUM_JOINTS> q0; //initial configuration
bool joint_ok_init; //initial configuraton flag
bool read_depth=false; 
double x_obj,y_obj,z_obj;

double px= 610.59326171875;
double py= 610.605712890625;
double u0= 320;
double v0= 240;
double inv_px=1.0/px;
double inv_py=1.0/py;

/*--------------------CALLBACK---------------------------*/
void readJointPos(const sensor_msgs::JointState jointStateMsg){
	if(!joint_ok_init){			
	    joint_ok_init=true;
			for(int i = 0; i < NUM_JOINTS; i++) {
					q0[i] = jointStateMsg.position[i];
		}	
	}            									
}
/*-------------------------------------------------------*/


ros::ServiceClient client;

cv_bridge::CvImagePtr cv_ptr;
int DEPTH_HEIGHT, DEPTH_WIDTH;

void readDepth(const sensor_msgs::ImageConstPtr& msg)
{
   
  if(!read_depth){  
  read_depth=true;
  cout<<"CALLBACK READ DEPTH"<<endl;
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
	 file.open("/home/yaskawa/catkin_visservo/src/add/visp_yaskawa/depth.txt");
	 
    for (int i =0; i<DEPTH_WIDTH; i++){
    	for (int j =0; j<DEPTH_HEIGHT; j++){
    		file<< (double)cv_ptr->image.at<short int>(cv::Point(i, j))/1000 <<" ";
    		}
    		file<< " \n";
    }

    file.close();
    }
}

void readObjectCenter(const std::string& obj) { 
    
	 visp_common::ChooseObject object_request;
	 object_request.request.object = obj;
    client.call(object_request);
	 int u, v;
	 
	 double mat[DEPTH_WIDTH][DEPTH_HEIGHT];
	 ifstream rfile("/home/yaskawa/catkin_visservo/src/add/visp_yaskawa/depth.txt");
	
	 while(!rfile.eof()){
		 for (int i =0; i<DEPTH_WIDTH; i++){
			for (int j =0; j<DEPTH_HEIGHT; j++){
				rfile >> mat[i][j];
				}
		}
	 }
	
	 rfile.close();
	
    if(object_request.response.success){
    u=object_request.response.object_center.x;
    v=object_request.response.object_center.y;
    
    double z=mat[u][v];
    cout<<"z: "<<z<<endl;
    x_obj = (u - u0)*z/px;
    y_obj = (v - v0)*z/py;
    cout<<"x_obj: "<<x_obj<<"  y_obj: "<<y_obj<<endl;
    }
    else {
    cout<<"Object not recognized!"<<endl;
    }
}



int main(int argc, char **argv){
    
   ros::init(argc, argv, "pre_visual_servoing_node");
   ros::NodeHandle n = ros::NodeHandle();
    

   double T_s=0.02;
   ros::Rate loop_rate(1/T_s);

   /*--------------------SUBSCRIBER & PUBLISHER---------------------------*/
   ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1, readJointPos);
   ros::Subscriber depth_sub= n.subscribe("/camera/aligned_depth_to_color/image_raw", 1, readDepth);
   ros::Publisher pub_joints_pose = n.advertise<geometry_msgs::PoseStamped>("/desired_pose", 1);
   ros::Publisher pub_joints_twist = n.advertise<geometry_msgs::TwistStamped>("/desired_twist", 1);
   ros::Publisher pub_stop = n.advertise<std_msgs::Bool>("stop_clik", 1);
   /*--------------------------------------------------------------------*/


   // SERVER CLIENTS
   //ros::ServiceClient client = nh.serviceClient<visp_yaskawa_msgs::ClikSetMode>("set_clik_mode_service");
   client = n.serviceClient<visp_common::ChooseObject>("chooseObject");
   
   //visp_yaskawa_msgs::ClikSetMode::Request clik_request;
   
 while(ros::ok() && (!joint_ok_init || !read_depth)){
		ros::spinOnce();
   }
    
   
    Matrix<4,4> n_T_c=Data(  -1, 0, -0.045, 0,
        								0.0012, -0.9659, -0.2588, 0.0633,
        							  -0.0044, -0.2588, 0.9659, 0.0969,
            						0,       0,      0,      1);
            									
    MotomanSIA5F yaskawa=MotomanSIA5F(n_T_c,2.0,"yaskawa"); 

    Vector<> q_DH= yaskawa.joints_Robot2DH(q0);
    Matrix<4,4> T_init = yaskawa.fkine(q_DH);

    Vector<3> pi=transl(T_init);
    
    readObjectCenter(argv[1]);
    
    //cout<<"x_obj: "<<x_obj<<"  y_obj: "<<y_obj<<endl;

    Vector<> pf_tilde=makeVector(x_obj,y_obj,0.0,1.0);
    Vector<> pf_b_tilde=T_init*pf_tilde;
    Vector<> pf_b=pf_b_tilde.slice(0,3);
    pf_b[2] = 0.40;
    //cout<<"pf_b: "<<pf_b[0]<<" "<<pf_b[1]<<" "<<pf_b[2]<<endl;
    
    Matrix<3,3> Rf= Data(-0.998975,-0.0450632,-0.0342326,
                         -0.0422114,0.968689,0.242885, 
                         -0.0169965,0.24403,-0.970464); 

    UnitQuaternion Qi(T_init);
    Vector<3> Qi_v=Qi.getV();
    double Qi_s=Qi.getS();
    UnitQuaternion Qf(Rf);
    Vector<3> Qf_v=Qf.getV();
    double Qf_s=Qf.getS();
    
    cout<<"Quaternione iniziale: "<<endl<<Qi<<endl;
    cout<<"Quaternione finale: "<<endl<<Qf<<endl;

    Quintic_Poly_Traj s_pos(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
       
    Line_Segment_Traj lin_traj( 
                             pi,//pi
                             pf_b,//pf
                             s_pos);
                             
    double time_now = ros::Time::now().toSec();
	 lin_traj.changeInitialTime(time_now);
	 s_pos.changeInitialTime(time_now);


	 while(ros::ok() && ((!lin_traj.isCompleate(time_now))))// || (!rot_traj.isCompleate(time_now))))
	 {
	    
		 time_now = ros::Time::now().toSec();

		 Vector<> final_position =lin_traj.getPosition(time_now);   		
     	 UnitQuaternion Q_now=Qi.interp(Qf,s_pos.getPosition(time_now),false);
     		
		 geometry_msgs::PoseStamped posemsg;
		 geometry_msgs::TwistStamped twistmsg;

		 posemsg.pose.position.x=final_position[0];
		 posemsg.pose.position.y=final_position[1];
		 posemsg.pose.position.z=final_position[2];
		
	    posemsg.pose.orientation.x=Q_now.getV()[0];
	    posemsg.pose.orientation.y=Q_now.getV()[1];
	    posemsg.pose.orientation.z=Q_now.getV()[2];
	    posemsg.pose.orientation.w=Q_now.getS();
		
	    
	   
		 twistmsg.twist.linear.x=0.0;
		 twistmsg.twist.linear.y=0.0;
		 twistmsg.twist.linear.z=0.0;
	
		 twistmsg.twist.angular.x=0.0;
		 twistmsg.twist.angular.x=0.0;
		 twistmsg.twist.angular.x=0.0;
		 
		 
		 //clik_request.request.mode=visp_yaskawa_msgs::ClikSetMode::Request::MODE_POSITION ;   

		 pub_joints_pose.publish(posemsg);
		 pub_joints_twist.publish(twistmsg);

		 ros::spinOnce();
		 loop_rate.sleep();
    }
    
    for(int i=0; i<20; i++){
		stop_traj.data = 1; // sending signal to stop clik                         
		pub_stop.publish(stop_traj);
		ros::spinOnce();
    }
    
    //clik_request.request.mode=visp_yaskawa_msgs::ClikSetMode::Request::MODE_STOP ;   

}

