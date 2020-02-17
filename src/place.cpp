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
    
   
                             
    Matrix<3,3> Rf= Data(-0.00754697, 0.273757, -0.962231,   //desired Rotation Matrix 
								  0.997673, -0.0632722, 0.0145066, 
								 -0.0679084, -0.959684, -0.27544);
                      
                          
    UnitQuaternion Qi(T_init);
    Vector<3> Qi_v=Qi.getV();
    double Qi_s=Qi.getS();
    UnitQuaternion Qf(Rf);
    Vector<3> Qf_v=Qf.getV();
    double Qf_s=Qf.getS();
    
    //cout<<"Quaternione iniziale: "<<endl<<Qi<<endl;
    //cout<<"Quaternione finale: "<<endl<<Qf<<endl;


    Quintic_Poly_Traj s_pos(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
    
    Vector<3> pf_absolute;
    Vector<3> pf1, pf2;
    
    pf1=pi+makeVector(-0.10,0.0,0.30);
    
    Line_Segment_Traj lin_traj1( 
		                          pi,//pi
		                          pf1,//pf
		                          s_pos);
        
    pf_absolute= makeVector(-0.40, -0.54, 0.40);
    
    Line_Segment_Traj lin_traj2( 
		                          pf1,//pi
		                          pf_absolute,//pf
		                          s_pos);                         
                             

    double time_now = ros::Time::now().toSec();
    lin_traj1.changeInitialTime(time_now);
	 lin_traj2.changeInitialTime(time_now+tf);
	 s_pos.changeInitialTime(time_now);


	 while(ros::ok() && ((!lin_traj1.isCompleate(time_now)) || (!lin_traj2.isCompleate(time_now))))
	 {
	    
		 time_now = ros::Time::now().toSec();
       Vector<3> final_position;
		 UnitQuaternion Q_now=Qi.interp(Qf,s_pos.getPosition(time_now),false);
       
       if((!lin_traj1.isCompleate(time_now))){
		 final_position=lin_traj1.getPosition(time_now);
		 }
       
       if(lin_traj1.isCompleate(time_now) && (!lin_traj2.isCompleate(time_now))){
		 final_position=lin_traj2.getPosition(time_now);}
		 

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

