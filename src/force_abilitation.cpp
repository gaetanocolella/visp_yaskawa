#include <ros/ros.h>
#include "TooN/TooN.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include "Robots/MotomanSIA5F.h"
#include "sensor_msgs/JointState.h"
#include "sun_systems_lib/TF/TF_INTEGRATOR.h"

#define NUM_JOINTS 7
#define DESIRED_FORCE -1.5
#define tool_weight -0.349 
#define _USE_MATH_DEFINES   

using namespace std;
using namespace TooN;

bool enabled=false;
Vector<3> forza=makeVector(0,0,0);  //force value
Matrix<3,1> F_sensore=Zeros(3);
Matrix<3,1> g_0 = Data(0.0,0.0,tool_weight);//m*g
Matrix<3,1> f_final=Zeros(3);
Vector<7> q_t_robot;
Vector<7> q_tDH;
Matrix<4> Teb;
Matrix<3> Rt;
Matrix<3> R_s_0;
Matrix<3> R_0_s;
Matrix<3> R_0_s_iniziale;

bool firstValue=true;
double f_z=0.0;
bool joint_ok_init;

void readJointPos(const sensor_msgs::JointState jointStateMsg)
   {     
     	if(!joint_ok_init){			
		joint_ok_init=true;
		for(int i = 0; i < NUM_JOINTS; i++) {
				q_t_robot[i] = jointStateMsg.position[i];
		}	
    }  
 	MotomanSIA5F yaskawa;
    q_tDH=yaskawa.joints_Robot2DH(q_t_robot); 
    Teb=yaskawa.fkine(q_tDH);
   
    Rt = Teb.slice<0, 0, 3, 3>(); 
    R_s_0=Rt*roty(-M_PI/2.0)*rotx(M_PI/2.0);
    R_0_s=R_s_0.T();

    if(firstValue){
                    R_0_s_iniziale=R_0_s;
                    firstValue=false;
                   }
    }

void readMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg){
	forza[0] = msg -> wrench.force.x;
    forza[1] = msg -> wrench.force.y;
    forza[2] = msg -> wrench.force.z;
	F_sensore=Data(forza[0],forza[1],forza[2]);
    f_final=F_sensore-R_0_s*g_0+R_0_s_iniziale*g_0;
	f_z=f_final[0][2];
}

void Callback_abilitation(const std_msgs::Bool& en){
		enabled=en.data;
}   
    
    
int main(int argc, char **argv){

	ros::init(argc, argv, "force_abilitation_node");
    ros::NodeHandle n = ros::NodeHandle();
    ros::NodeHandle n_private = ros::NodeHandle("~");
    
	double hz = 1000.0;
	double T_s=1.0/hz;
    ros::Rate loop_rate(hz);
       
    ros::Subscriber rft_sensor_sub=n.subscribe("rft_data",1,readMeasuredWrench);
    ros::Subscriber sub_en=n.subscribe("fc_abilitation",1,Callback_abilitation);
	ros::Subscriber sub =n.subscribe("joint_states",1,readJointPos);
    ros::Publisher pub = n.advertise<std_msgs::Float64>("pos_correction", 1);

    double fd;
	n_private.param("desired_force", fd, DESIRED_FORCE);
    
	double controller_gain=0.1;
	TF_INTEGRATOR tf=TF_INTEGRATOR(T_s,controller_gain);

    double d_z=0.0;
    std_msgs::Float64 msg;
    
    bool info=false;
	while(ros::ok() && !enabled){
		if(!info){
			cout<<"[Force Controller]: Waiting for abilitation signal..."<<endl;
			info=true;
		}
		ros::spinOnce();
	}
	
	cout<<"[Force Controller]: Enabled."<<endl;

    int reached;
	bool reset=false;
   	while(ros::ok()){
		reached =0;   
		ros::spinOnce();
		if(enabled){
			if(fabs(fd-f_z)<0.1){
				f_z=fd;
			}
			if(f_z<=fd) reached=1;
			d_z=tf.apply(fd-f_z);
			reset=true;
			cout<<"[Force Controller]: Correcting."<<endl;
		}
		else if(!enabled){
			reached=1;
			if(reset){
				tf.reset();
				reset=false;
				cout<<"[Force Controller]: Stopped and resetted."<<endl;
			}
			d_z=0.0;
		}
		
		
		msg.data=d_z;
		pub.publish(msg);
		loop_rate.sleep();
		
		
    }

}
