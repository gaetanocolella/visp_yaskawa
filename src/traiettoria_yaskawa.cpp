#include "ros/ros.h"
#include "std_msgs/String.h"
#include "TooN/TooN.h"
#include <TooN/SVD.h>
#include "math.h"
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include <sensor_msgs/JointState.h>
#include <iostream>
#include "UnitQuaternion.h"
#include "RobotLinkRevolute.h"
#include "Robot.h"
#include <math.h>
#include "PortingFunctions.h"
#include "GeometryHelper.h"
#include "AngVec.h"
#include <Robots/MotomanSIA5F.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

#define NUM_JOINTS 7
#define DIM_R 3
#define DIM_T 4
#define _USE_MATH_DEFINES

using namespace std;
using namespace TooN;

sensor_msgs::JointState current_joint_position;
int prova=1;
Vector<NUM_JOINTS> q0;
//makeVector(-1.5475890636444092, 0.32370030879974365, -0.02413361705839634, 0.29617977142333984, -0.0076243397779762745, -1.5202265977859497, 1.570042371749878);
//rostopic pub joint_states sensor_msgs/JointState '{position: [-1.3859895467758179, 0.5565312504768372, -0.05192752555012703, 0.6827732920646667, 0.0031894647981971502, -1.625778317451477, 1.1097440719604492]}'
Vector<3> pf; //posizione finale desiderata
Matrix<DIM_T,DIM_T> T_curr;
Vector<3> b_twist_f_linear;
Vector<3> b_twist_f_angular;
Vector<6> veld;
Vector <NUM_JOINTS> qDH_k;

Matrix<DIM_T,DIM_T> n_T_c=Data( -1, 0, -0.045, 0,
        									0.0012, -0.9659, -0.2588, 0.0633,
        									-0.0044, -0.2588, 0.9659, 0.0969,
            									0,       0,      0,      1);
            									
MotomanSIA5F yaskawa=MotomanSIA5F(n_T_c,2.0,"yaskawa");
 
bool Twist_arrived=false;
 
void readJointPos(const sensor_msgs::JointState jointStateMsg)
{
    if (jointStateMsg.position.size() == NUM_JOINTS){
		for(int i = 0; i < NUM_JOINTS; i++) {
				q0[i] = jointStateMsg.position[i];
		}	
		q0=yaskawa.joints_Robot2DH(q0);
	   T_curr=yaskawa.fkine(q0);
      }
      
      cout<<"T_curr: "<<endl<<T_curr<<endl;
}

void readTwist(const geometry_msgs::Twist msg)
{
   Matrix <DIM_R,DIM_R> R_curr=(T_curr.slice<0,0,3,3>()).T();
   Vector<3> c_twist_f_linear;
   Vector<3> c_twist_f_angular;
   
   c_twist_f_linear[0] = msg.linear.x;
   c_twist_f_linear[1] = msg.linear.y;
   c_twist_f_linear[2] = msg.linear.z;
   c_twist_f_angular[0] = msg.angular.x;
   c_twist_f_angular[1] = msg.angular.y;
   c_twist_f_angular[2] = msg.angular.z;
      		
   b_twist_f_linear=R_curr*c_twist_f_linear;
   b_twist_f_angular=R_curr*c_twist_f_angular;
   		
   for(int i = 0; i < 3; i++) {
   veld[i] = b_twist_f_linear[i];
   cout<<"c_twist_f_linear: "<<c_twist_f_linear[i]<<endl;
   cout<<"b_twist_f_linear: "<<b_twist_f_linear[i]<<endl;}
   for(int i = 3; i < 6; i++) {
   veld[i] = b_twist_f_angular[i-3];
   cout<<"----------------------------------------------------------------"<<endl;
   cout<<"c_twist_f_angular: "<<c_twist_f_angular[i-3]<<endl;
   cout<<"b_twist_f_angular: "<<b_twist_f_angular[i-3]<<endl;
   }

 Twist_arrived= true;

}

int main(int argc, char **argv)
{

  double T_s=0.002;    
  ros::init(argc, argv, "traiettoria_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(1/T_s);
  ros::Subscriber joint_states_sub= n.subscribe("joint_states", 1, readJointPos);
  ros::Subscriber twist_sub = n.subscribe("/visp_twist", 1, readTwist);
  ros::Publisher joint_states_pub = n.advertise<std_msgs::Float64MultiArray>("joint_ll_control", 1);
  //ros::Publisher joint_states_pub = n.advertise<std_msgs::Float64MultiArray>("test_q", 1);

  double secondary_gain=1; 
  double gain=2;

  Vector<NUM_JOINTS> qrobot_k,qprobot;
  Vector<> qpDH = makeVector(0,0,0,0,0,0,0);
  Vector<NUM_JOINTS> pesi = makeVector(1,1,1,4,1,1,1);

  Matrix<6, NUM_JOINTS> J;
 
    while(Twist_arrived==false){
    ros::spinOnce();
          }             
  joint_states_sub.shutdown();
  qDH_k=q0;
  cout<<"qDH_k:"<<qDH_k<<endl;
  Matrix<4,4> T_clik;
   
     
while (ros::ok() ){
  
   J=yaskawa.jacob_geometric(qDH_k,NUM_JOINTS);
   cout<<"veld: "<<veld<<endl;
   T_clik=yaskawa.fkine(qDH_k);
   cout<<"posizione: "<<T_clik[0][3]<<" "<<T_clik[1][3]<<" "<<T_clik[2][3]<<endl;
   qDH_k=yaskawa.clik( 
    						qDH_k, // Giunti attuali
    						Zeros(6), //Errore in posizione
    						J, //Jacobiano: Deve essere quello geomentrico o analitico a seconda dei casi! CONTROLLARE!
    						veld, // Velocità desiderata
    						gain, //guadagno del clik
    						T_s, // Passo di campionamento: DEVE ESSERE UGUALE AL LOOP RATE!!!!
    						secondary_gain, // Guadagno dell'obj secondario (se volete metterlo) 
    						// qsegn, NO! QUESTO NON VA BENE!
    						yaskawa.grad_fcst_target_configuration( // Questa funzione calcola le velocità di giuto per andare verso una config desiderata
                             qDH_k, // Configurazione attuale
                             makeVector(0.0,0.0,0.0,M_PI/2.0,0.0,0.0,0.0), //<- configurazione desiderata in DH -> consiglio centro corsa in convenzione DH che per 	 																											lo yaksawa sicuramente non può essere tutto zero! DA CONTROLLARE! 
                             Ones(7) // Pesi da dare per ogni giunto in questa funzione
                     ),
    						qpDH // Le velocità di giunto un USCITA dal clik
    						);
    						
    qrobot_k=yaskawa.joints_DH2Robot(qDH_k); //Passo da convenzione DH a robot
    qprobot=yaskawa.jointsvel_DH2Robot(qpDH);
    
   vector<bool> check_joint_lim=yaskawa.checkHardJointLimits(qrobot_k);
   vector<bool>  check_vel_lim=yaskawa.checkHardVelocityLimits(qprobot);

   for(int i=0;i<NUM_JOINTS;i++)
    {
         if(check_joint_lim[i])
        {
           cout<<" Il giunto "<<i+1<<" ha superato il limite, valore "<<qrobot_k[i]<<endl;
           return -1;
        }
           if(check_vel_lim[i])
        {
           cout<<" Il giunto "<<i+1<<" ha superato il limite di velocità, valore "<<qprobot[i]<<endl;
           return -1;
        }
    }

   // std::cout<<qrobot_k<<std::endl;

    std_msgs::Float64MultiArray joints_states_msg;
    joints_states_msg.data.resize(NUM_JOINTS*2);
    
    for (int i=0;i<NUM_JOINTS;i++)
    {
	   joints_states_msg.data[i]=qrobot_k[i];
		joints_states_msg.data[i+NUM_JOINTS]=0.0;
    }

    //cout<<"msg: "<<joints_states_msg<<endl;
    joint_states_pub.publish(joints_states_msg);
    
    ros::spinOnce();
    loop_rate.sleep();

  }

  Matrix<4,4> Tf;
  Tf=yaskawa.fkine(qDH_k);
  cout<<"posizione finale raggiunta "<<Tf[0][3]<<" "<<Tf[1][3]<<" "<<Tf[2][3]<<endl;
}
