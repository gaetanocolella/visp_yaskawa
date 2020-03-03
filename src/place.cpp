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

using namespace std;
using namespace TooN;

#define tf 20.0
#define NUM_JOINTS 7

std_msgs::Bool stop_traj;
Vector<NUM_JOINTS> q0; //initial configuration
bool joint_ok_init = false; //initial configuraton flag


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


int main(int argc, char **argv){
    
   ros::init(argc, argv, "place_node");
   ros::NodeHandle n = ros::NodeHandle();
    

   double T_s=0.02;
   ros::Rate loop_rate(1/T_s);

   /*--------------------SUBSCRIBER & PUBLISHER---------------------------*/
   ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1, readJointPos);
   ros::Publisher pub_joints_pose = n.advertise<geometry_msgs::PoseStamped>("/desired_pose", 1);
   ros::Publisher pub_joints_twist = n.advertise<geometry_msgs::TwistStamped>("/desired_twist", 1);
   ros::Publisher pub_stop = n.advertise<std_msgs::Bool>("stop_clik", 1);
   /*--------------------------------------------------------------------*/

   
 while(ros::ok() && !joint_ok_init){
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
    
    
    Matrix<3,3> Rf = Data(-0.086954, 0.252469, -0.967364, 
                          0.9826, 0.181139, -0.00146728, 
                          0.164187, -0.950467, -0.257236);
    
    Matrix<3,3> Rf_inter = Data(-0.851953, 0.454978, -0.293704,    //intermediate Rotation Matrix
								 0.477294, 0.878295, -0.00773122, 
								 0.215377, -0.146745, -0.9568936);                            
                             
    Matrix<3,3> Rf_final= Data( 0.999899, 0.00655554, 0.053824,   //desired Rotation Matrix 
							   -0.0112276, -0.259002, 0.965514, 
								0.00981878, -0.965824, -0.258554);
								                           
    UnitQuaternion Qi(T_init);
    Vector<3> Qi_v=Qi.getV();
    double Qi_s=Qi.getS();
    
    UnitQuaternion Qf(Rf);
    Vector<3> Qf_v=Qf.getV();
    double Qf_s=Qf.getS();
    
    UnitQuaternion Qf_inter(Rf_inter);
    Vector<3> Qf_inter_v=Qf_inter.getV();
    double Qf_inter_s=Qf_inter.getS();
    
    UnitQuaternion Qf_final(Rf_final);
    Vector<3> Qf_final_v=Qf_final.getV();
    double Qf_final_s=Qf_final.getS();
    
    //cout<<"Quaternione iniziale: "<<endl<<Qi<<endl;
    //cout<<"Quaternione finale: "<<endl<<Qf<<endl;


    Quintic_Poly_Traj s_pos(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
    Quintic_Poly_Traj s_pos1(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
    Quintic_Poly_Traj s_pos2(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
    Quintic_Poly_Traj s_pos_final(   
                            		10.0,//duration
                            		0.0,//double initial_position,
                            		1.0);//double final_position,
    
    Vector<3> pf_absolute;
    Vector<3> pf1, pf2;
    
    pf1=pi+makeVector(-0.10, 0.0, 0.15);
    
    Line_Segment_Traj lin_traj1( 
		                          pi,//pi
		                          pf1,//pf
		                          s_pos);
		                          
    pf2 = makeVector(-0.408989,-0.00601007,0.801611);
    
	string obj="";
	obj=argv[1];
	
	if(obj=="denkmit"){
    		pf_absolute= makeVector(-0.00958789,0.45,0.561147);}
	else if(obj=="finish"){
   			pf_absolute= makeVector(-0.10958789,0.45,0.550147);}
	else if(obj=="beckmann"){
			pf_absolute= makeVector(-0.20958789,0.45,0.550147);
	}
	else if(obj=="balea"){
			pf_absolute= makeVector(-0.20958789,0.45,0.550147);
	}
	else{
		cout<<"Object name is not correct!"<<endl;
		for(int i=0; i<20; i++){
		stop_traj.data = 1; // sending signal to stop clik                         
		pub_stop.publish(stop_traj);
		ros::spinOnce();
    }
	}
    
    Line_Segment_Traj lin_traj2( 
		                          pf1,//pi
		                          pf2,//pf
		                          s_pos);                         
    
    Line_Segment_Traj lin_traj3( 
		                          pf2,//pi
		                          pf_absolute,//pf
		                          s_pos); 
	
	Line_Segment_Traj lin_traj_final( 
		                          pf_absolute,//pi
		                          pf_absolute+makeVector(0.0, 0.15, 0.0),//pf
		                          s_pos_final); 	                          

    double time_now = ros::Time::now().toSec();
    lin_traj1.changeInitialTime(time_now);
	lin_traj2.changeInitialTime(time_now+tf);
	lin_traj3.changeInitialTime(time_now+(2*tf));
	lin_traj_final.changeInitialTime(time_now+(3*tf));
	s_pos.changeInitialTime(time_now);
	s_pos1.changeInitialTime(time_now+tf);
    s_pos2.changeInitialTime(time_now+(2*tf));

	 while(ros::ok() && ((!lin_traj1.isCompleate(time_now)) || (!lin_traj2.isCompleate(time_now)) || (!lin_traj3.isCompleate(time_now)) || (!lin_traj_final.isCompleate(time_now))))
	 {
	    
		 time_now = ros::Time::now().toSec();
       Vector<3> final_position;
		 UnitQuaternion Q_now;
       
       if((!lin_traj1.isCompleate(time_now))){
		 final_position=lin_traj1.getPosition(time_now);
         Q_now=Qi.interp(Qf_inter,s_pos.getPosition(time_now),true);}
       
       if(lin_traj1.isCompleate(time_now) && (!lin_traj2.isCompleate(time_now))){
		 final_position=lin_traj2.getPosition(time_now);
		 Q_now=Qf_inter.interp(Qf,s_pos1.getPosition(time_now),true);}
		 
	   if(lin_traj2.isCompleate(time_now) && (!lin_traj3.isCompleate(time_now))){
		 final_position=lin_traj3.getPosition(time_now);
		 Q_now=Qf.interp(Qf_final,s_pos2.getPosition(time_now),true);}
		 
	   if(lin_traj3.isCompleate(time_now) && (!lin_traj_final.isCompleate(time_now))){
		 final_position=lin_traj_final.getPosition(time_now);
		 Q_now=Qf_final;}
		 
		geometry_msgs::PoseStamped posemsg;
		geometry_msgs::TwistStamped twistmsg;

        cout<<"pf: "<<final_position[0]<<" "<<final_position[1]<<" "<<final_position[2]<<endl;
       
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
    

}

