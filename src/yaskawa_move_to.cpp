#include "ros/ros.h"
#include "TooN/TooN.h"
#include "Robots/MotomanSIA5F.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h> 
#include "Traj_Generators/Vector_Independent_Traj.h"
#include "Traj_Generators/Quintic_Poly_Traj.h"

using namespace std;
using namespace TooN;


Vector<7> q0,qf;
           								
double tf=25.0;
bool joint_ok_init;

//Oggetto robot
MotomanSIA5F yaskawa;
	
	
void readJointPos(const sensor_msgs::JointState jointStateMsg)
{
 	if(!joint_ok_init){			
		joint_ok_init=true;
		for(int i = 0; i < yaskawa.getNumJoints(); i++) {
				q0[i] = jointStateMsg.position[i];
		}	
    }           									
}

int main(int argc, char*argv[]){

	ros::init(argc, argv, "yaskawa_move_to");

    ros::NodeHandle nh;
   
	
	qf=makeVector(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]));
	
    joint_ok_init=false;
    
    
	ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 1, readJointPos);
	ros::Publisher joint_states_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_ll_control", 1);
	
	while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}
	
	
	//GENERAZIONE TRAIETTORIA
	Vector_Independent_Traj traj;	
	
	for(int i=0;i<yaskawa.getNumJoints();i++){
	traj.push_back_traj(Quintic_Poly_Traj (tf, q0[i], qf[i]));
	}
	
	double hz = 1000.0;
    ros::Rate loop_rate(hz);

    double time_now = ros::Time::now().toSec();
    
    traj.changeInitialTime(time_now);
    
    std_msgs::Float64MultiArray joints_states_msg;
    joints_states_msg.data.resize(yaskawa.getNumJoints()*2);
    
    
    while(ros::ok() && (!traj.isCompleate(time_now))){
    
    time_now = ros::Time::now().toSec();
    
    Vector<> position =traj.getPosition(time_now);
    Vector<> velocity =traj.getVelocity(time_now);
    
   vector<bool> check_joint_lim=yaskawa.checkHardJointLimits(position);
   vector<bool>  check_vel_lim=yaskawa.checkHardVelocityLimits(velocity);

   for(int i=0;i<yaskawa.getNumJoints();i++)
    {
         if(check_joint_lim[i])
        {
           cout<<" Il giunto "<<i+1<<" ha superato il limite, valore "<<position[i]<<endl;
           return -1;
        }
           if(check_vel_lim[i])
        {
           cout<<" Il giunto "<<i+1<<" ha superato il limite di velocità, valore "<<velocity[i]<<endl;
           return -1;
        }
    }
    
    
    for (int i=0;i<yaskawa.getNumJoints();i++)
    {
	joints_states_msg.data[i]=position[i];
	joints_states_msg.data[i+yaskawa.getNumJoints()]=velocity[i];
    }
   
    joint_states_pub.publish(joints_states_msg);
	
	loop_rate.sleep();
	
}

}
