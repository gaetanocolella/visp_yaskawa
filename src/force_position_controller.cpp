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


	Vector<NUM_JOINTS> q0; //initial configuration
	bool joint_ok_init=false;
	std_msgs::Bool fc_abilit;
   double d_z=0.0; //position correction
   Vector<3> delta_z;    
   Vector<7> pos_i_Robot;
	Vector<3> pos;
	Vector<3> vel;
	UnitQuaternion quat;
	Vector<3> w;
		
   
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
		 delta_z=makeVector(0.0,0.0,d_z);
		 cout<<"delta_z :"<<delta_z<<endl;
	}
	
	/*--------------------------------------------------------*/
	
	
	   int main(int argc, char **argv){
	   
	   
	ros::init(argc, argv, "force_position_control_node");
   ros::NodeHandle n = ros::NodeHandle();
	   
	/*--------------------SUBSCRIBER & PUBLISHER---------------------------*/
   ros::Publisher fc_abilitation_pub = n.advertise<std_msgs::Bool>("fc_abilitation", 1);
	ros::Subscriber pos_correction_sub=n.subscribe("pos_correction",1,Callback_correction);
	ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1, readJointPos);
		ros::Publisher joint_states_pub = n.advertise<std_msgs::Float64MultiArray>("joint_ll_control", 1);
    /*--------------------------------------------------------------------*/
    
   
   double T_s=0.001;
   double hz=1.0/T_s;
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
   UnitQuaternion oldQ(T_init);
   Vector<6> error = Ones; 
  
  
  	pos=transl(T_init);
  	cout<<"posizione iniziale: "<<pos<<endl;
	quat=oldQ;
	vel=Zeros(yaskawa.getNumJoints());
	w=Zeros(yaskawa.getNumJoints());
   
   cout<<"PROVA PUBLISH" <<endl;
   
   /*for(int i=0;i<3;i++){
   fc_abilit.data = 1; // sending signal to enable the force controller node                          
   fc_abilitation_pub.publish(fc_abilit);
   ros::spinOnce();
   loop_rate.sleep();
   }*/

       
       while(ros::ok()){
			
			cout<<"pos: "<<pos<<" "<<pos+delta_z<<endl;
			
		        qDH_k = yaskawa.clik(   
                                qDH_k, //<- qDH attuale
                                pos+delta_z, // <- posizione desiderata
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
    
    
    Matrix<4,4> Tf;
    Tf=yaskawa.fkine(qDH_k);
    cout<<"Tf: "<<endl<<Tf;
    cout<<"Posizione finale raggiunta: "<<Tf[0][3]<<" "<<Tf[1][3]<<" "<<Tf[2][3]<<endl;   
    
    
     
    ros::spinOnce();
    loop_rate.sleep();
    }
 }    
    
    
    
    
    
    
    
