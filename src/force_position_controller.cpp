#include <ros/ros.h>
#include "TooN/TooN.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include "sensor_msgs/JointState.h"
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "Traj_Generators/Line_Segment_Traj.h"
#include "Robots/MotomanSIA5F.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

using namespace std;
using namespace TooN;

#define NUM_JOINTS 7
#define tf 1.0

	Vector<NUM_JOINTS> q0; //initial configuration
	bool joint_ok_init=false;
   double d_z=0.0; //position correction
   Vector<3> delta_z=Zeros;
	Vector<3> pos;
	Vector<3> vel;
	UnitQuaternion quat;
	Vector<3> w;
	bool stop_clik = false;
		
   
	/*--------------------CALLBACK---------------------------*/
	void readJointPos(const sensor_msgs::JointState jointStateMsg)
	{
	 	if(!joint_ok_init){			
			joint_ok_init=true;
			for(int i = 0; i < NUM_JOINTS; i++) {
					q0[i] = jointStateMsg.position[i];

			}	
			 cout<<"q0: "<<q0<<endl;
		 }
		            									
	}
	
	void Callback_correction(const std_msgs::Float64& msg){
		 d_z=msg.data;
		 if(d_z == 0.0){
		 	stop_clik = true;		 
		 }
		 delta_z=makeVector(0.0,0.0,d_z);
		 cout<<"delta_z: "<<d_z<<endl;
	}
	
	/*--------------------------------------------------------*/
	
	
	 int main(int argc, char **argv){
	   
	   
	ros::init(argc, argv, "force_position_control_node");
   ros::NodeHandle n = ros::NodeHandle();
	   
	/*--------------------SUBSCRIBER & PUBLISHER---------------------------*/
	ros::Subscriber pos_correction_sub=n.subscribe("pos_correction",1,Callback_correction);
	ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1, readJointPos);
   ros::Publisher joint_states_pub = n.advertise<std_msgs::Float64MultiArray>("joint_ll_control", 1);
    /*--------------------------------------------------------------------*/
    
   
   double hz=50.0;
   double T_s=1.0/hz;
   ros::Rate loop_rate(1/T_s);
    

   while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}
	
	 Matrix<4,4> n_T_e= Data(-1.0, 0.0, 0.0, 0.0,
                             0.0,-1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.2369,
                             0.0, 0.0, 0.0, 1); 
                           
                               
    MotomanSIA5F yaskawa(n_T_e,2.0,"yaskawa");
   
    Vector<> q_DH= yaskawa.joints_Robot2DH(q0);
    Matrix<4,4> T_init = yaskawa.fkine(q_DH);
    
   //Inizializzazione variabili
   Vector<> qDH_k = q_DH;
   Vector<> qpDH = Zeros(yaskawa.getNumJoints());
   UnitQuaternion oldQ;
   Vector<6> error = Ones; 
   
   cout<<"fkine: "<<endl<<T_init<<endl;
   
   
   Vector<3> pi=transl(T_init);
    

    //cout<<"pf_b: "<<pf_b[0]<<" "<<pf_b[1]<<" "<<pf_b[2]<<endl;
    
    
    Matrix<3,3> Rf= Data(-0.998797, -0.0398641, 0.0285415,
                         -0.0405092, 0.998928, -0.0223936, 
                         -0.0276183, -0.0235229, -0.999342); 


    UnitQuaternion Qi(T_init);
    UnitQuaternion Qf(Rf);
    pos=transl(T_init);
    oldQ=Qi;
    
    Quintic_Poly_Traj s_pos(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,

   /* 
       
    Line_Segment_Traj lin_traj( 
                             pi,//pi
                             pi+makeVector(0.0,0.0,d_z),//pf
                             s_pos);
                             

	 lin_traj.changeInitialTime(time_now);
	
 */
      double time_now = ros::Time::now().toSec();

    s_pos.changeInitialTime(time_now);
	 vel=Zeros(yaskawa.getNumJoints());
	 w=Zeros(yaskawa.getNumJoints());
    Matrix<4,4> Tf;
           
      while(ros::ok() && stop_clik==false){
		
		
				 /*pos =lin_traj.getPosition(time_now);*/
				 time_now = ros::Time::now().toSec();
				 UnitQuaternion quat=Qi.interp(Qf,s_pos.getPosition(time_now),false);
				 cout<<"pi: "<<pi<<endl;
             Vector<3> pf=pos+delta_z;
             
		        qDH_k = yaskawa.clik(   
                                qDH_k, //<- qDH attuale
                                pf, // <- posizione desiderata
                                quat, // <- quaternione desiderato
                                oldQ,// <- quaternione al passo precedente (per garantire la continuità)
                                vel, // <- velocità in translazione desiderata
                                w, //<- velocità angolare deisderata
                                Ones,// <- maschera, se l'i-esimo elemento è zero allora l'i-esima componente cartesiana non verrà usata per il calcolo dell'errore
                                0.002*hz,// <- guadagno del clik (quì è scelto in maniera conservativa)
                                1.0/hz,// <- Ts, tempo di campionamento
                                0.0, // <- quadagno obj secondario
                                Zeros(yaskawa.getNumJoints()), // velocità di giunto dell'obj secondario (qui sono zero)              
                                //Return Vars
                                qpDH, // <- variabile di ritorno velocità di giunto
                                error, //<- variabile di ritorno errore
                                oldQ // <- variabile di ritorno: Quaternione attuale (N.B. qui uso oldQ in modo da aggiornare direttamente la variabile oldQ e averla già pronta per la prossima iterazione)
                            );
                            
         cout<<"pf: "<<pf<<endl;

        // Acluni cout se servono...
        //cout << "posd: " << pos << endl;
        //cout << "pos: " << transl( iiwa.fkine(qDH_k) ) << endl;
        //cout << "qDH_k: " << qDH_k << endl;
        //cout << "qpDH: " << qpDH << endl;
        //cout << "norm error: " << norm(error) << endl;
        //cout << "q_dot: " << qpDH << endl;

        //Calcolo q in robot convention
        Vector<> qR = yaskawa.joints_DH2Robot( qDH_k );
        Vector<> qpR = yaskawa.jointsvel_DH2Robot(qpDH);

        //check limits
        if( yaskawa.exceededHardJointLimits( qR ) ){
            //stampo a schermo i giunti incriminati
            cout << "ERROR ROBOT JOINT LIMITS!! On joints:" << endl;
            cout << yaskawa.jointsNameFromBitMask( yaskawa.checkHardJointLimits(qR) ) << endl;
            exit(-1); //esco
        }
        if( yaskawa.exceededHardVelocityLimits( yaskawa.jointsvel_DH2Robot(qpDH) ) ){
            //stampo a schermo i giunti incriminati
            cout << "ERROR ROBOT Velocity!! On joints:" << endl;
            cout << yaskawa.jointsNameFromBitMask( yaskawa.checkHardVelocityLimits( yaskawa.jointsvel_DH2Robot(qpDH) ) ) << endl;
            exit(-1); //esco
        }


    std_msgs::Float64MultiArray joints_states_msg;
    joints_states_msg.data.resize(yaskawa.getNumJoints()*2);
    
    for (int i=0;i<yaskawa.getNumJoints();i++)
    {
	   joints_states_msg.data[i]=qR[i];
		joints_states_msg.data[i+yaskawa.getNumJoints()]=qpR[i];
    }

    joint_states_pub.publish(joints_states_msg);
    
    
    Tf=yaskawa.fkine(qDH_k);
    
    if(Tf[2][3]<0.01){
    		cout<<"Limite z!"<<endl;
	 		break;
	 }
 
    ros::spinOnce();
    loop_rate.sleep();
    }
     cout<<"Posizione finale raggiunta: "<<Tf[0][3]<<" "<<Tf[1][3]<<" "<<Tf[2][3]<<endl;   
 }    
    
    
    
    
    
    
    
