#include <ros/ros.h>
#include "TooN/TooN.h"
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include "sensor_msgs/JointState.h"
#include <std_msgs/Bool.h>
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "Traj_Generators/Line_Segment_Traj.h"
#include "Robots/MotomanSIA5F.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

using namespace std;
using namespace TooN;

#define _USE_MATH_DEFINES
#define NUM_JOINTS 7
#define DISPLACEMENT_Z 0.15


Vector<NUM_JOINTS> q0; //initial configuration
double d_z=0.0; //position correction
bool enable_force=false;
bool joint_ok_init;

/*--------------------CALLBACK---------------------------*/
void readJointPos(const sensor_msgs::JointState jointStateMsg)
{
 	if(!joint_ok_init){			
		joint_ok_init=true;
		for(int i = 0; i < NUM_JOINTS; i++) {
				q0[i] = jointStateMsg.position[i];
		}	
    }           									
}

void Callback_correction(const std_msgs::Float64& msg){
    d_z=msg.data;
}
/*-------------------------------------------------------*/


std_msgs::Bool fc_abilit;


int main(int argc, char **argv){

	ros::init(argc, argv, "force_position_control_node");
    ros::NodeHandle n = ros::NodeHandle();
    ros::NodeHandle n_private = ros::NodeHandle("~");
    
    
    double T_s=0.001;
    ros::Rate loop_rate(1/T_s);
    
    
   /*--------------------SUBSCRIBER & PUBLISHER---------------------------*/
    ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1, readJointPos);
    ros::Subscriber pos_correction_sub=n.subscribe("pos_correction",1,Callback_correction);
    ros::Publisher fc_abilitation_pub = n.advertise<std_msgs::Bool>("fc_abilitation", 1);
    ros::Publisher pub_joints_pose = n.advertise<geometry_msgs::PoseStamped>("/desired_pose", 1);
	ros::Publisher pub_joints_twist = n.advertise<geometry_msgs::TwistStamped>("/desired_twist", 1);
    /*--------------------------------------------------------------------*/
    
   
	while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}
	
	
    Matrix<4,4> n_T_c=Data( -1, 0, -0.045, 0,
        							0.0012, -0.9659, -0.2588, 0.0633,
        							-0.0044, -0.2588, 0.9659, 0.0969,
            							  0,       0,      0,      1);
            									
    MotomanSIA5F yaskawa=MotomanSIA5F(n_T_c,2.0,"yaskawa");  

    Vector<> q_DH= yaskawa.joints_Robot2DH(q0);
    Matrix<4,4> T_init = yaskawa.fkine(q_DH);
    Vector<3> pi=transl(T_init);
    Vector<> pf_tilde=makeVector(0.0,0.0,DISPLACEMENT_Z,1.0);
    Vector<> pf_b_tilde=T_init*pf_tilde;
    Vector<> pf_b=pf_b_tilde.slice(0,3);

    UnitQuaternion Q(T_init);
    Vector<3> Q_v=Q.getV();
    double Q_s=Q.getS();

    Quintic_Poly_Traj s_pos(   
                            10.0,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
       
    Line_Segment_Traj lin_traj( 
                             pi,//pi
                             pf_b,//pf
                             s_pos);
                                         
    double time_now = ros::Time::now().toSec();
	lin_traj.changeInitialTime(time_now);

	while(ros::ok() && (!lin_traj.isCompleate(time_now))){
	
    ros::spinOnce();
    
    time_now = ros::Time::now().toSec();

    fc_abilit.data = 1; // sending signal to enable the force controller node
    fc_abilitation_pub.publish(fc_abilit);
    Vector<> final_position =lin_traj.getPosition(time_now)+makeVector(0.0,0.0,d_z);
    //Vector<> final_velocity =lin_traj.getVelocity(time_now); 

    geometry_msgs::PoseStamped posemsg;
    geometry_msgs::TwistStamped twistmsg;

        posemsg.pose.position.x=final_position[0];
		posemsg.pose.position.y=final_position[1];
		posemsg.pose.position.z=final_position[2];
		
		posemsg.pose.orientation.x=Q_v[0];
		posemsg.pose.orientation.y=Q_v[1];
		posemsg.pose.orientation.z=Q_v[2];
		posemsg.pose.orientation.w=Q_s;

        twistmsg.twist.linear.x=0.0;
		twistmsg.twist.linear.y=0.0;
		twistmsg.twist.linear.z=0.0;
		
		twistmsg.twist.angular.x=0.0;
		twistmsg.twist.angular.x=0.0;
		twistmsg.twist.angular.x=0.0;
		
		posemsg.header.stamp=ros::Time::now();
		posemsg.header.frame_id="base_link";
         
          
        pub_joints_pose.publish(posemsg);
        pub_joints_twist.publish(twistmsg);

    loop_rate.sleep();

    }
    
	}