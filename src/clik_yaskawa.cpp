#include "ros/ros.h"
#include "Robots/MotomanSIA5F.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

bool joint_ok_init;
bool ok;

using namespace TooN;
using namespace std;

Vector<7> pos_i_Robot;

Vector<3> pos;
Vector<3> vel;
UnitQuaternion quat;
Vector<3> w;

Matrix<4,4> T_init;

Matrix<4,4> n_T_c=Data( -1, 0, -0.045, 0,
        		       0.0012, -0.9659, -0.2588, 0.0633,
        			   -0.0044, -0.2588, 0.9659, 0.0969,
            				0,       0,      0,      1);
            									
MotomanSIA5F yaskawa=MotomanSIA5F(n_T_c,2.0,"yaskawa");


void sub_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg){
	
	if(!joint_ok_init){			
		joint_ok_init=true;
		pos_i_Robot[0]=msg->position[0];
		pos_i_Robot[1]=msg->position[1];
		pos_i_Robot[2]=msg->position[2];
		pos_i_Robot[3]=msg->position[3];
		pos_i_Robot[4]=msg->position[4];
		pos_i_Robot[5]=msg->position[5];
		pos_i_Robot[6]=msg->position[6];
				
	}

}

void sub_desired_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

		pos[0]=msg->pose.position.x;
		pos[1]=msg->pose.position.y;
		pos[2]=msg->pose.position.z;
		
		Vector<3> vect;
		vect[0]=msg->pose.orientation.x;
		vect[1]=msg->pose.orientation.y;
		vect[2]=msg->pose.orientation.z;
		double scalar=msg->pose.orientation.w;
		
		quat=UnitQuaternion(scalar,vect);
}



void sub_desired_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
		
		vel[0]=msg->twist.linear.x;
		vel[1]=msg->twist.linear.y;
		vel[2]=msg->twist.linear.z;
			
		w[0]=msg->twist.angular.x;
		w[1]=msg->twist.angular.y;
		w[2]=msg->twist.angular.z;
}


int main(int argc, char*argv[]){
	
	ros::init(argc, argv, "Clik");

    ros::NodeHandle nh;
    
    joint_ok_init=false;
   
	yaskawa.setDLSJointSpeedSaturation(5.0);
	ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("/joint_command2", 1);
	ros::Subscriber sub = nh.subscribe("/joint_states", 1, sub_joint_state_cb);
	ros::Subscriber sub_pose=nh.subscribe("/desired_pose",1,sub_desired_pose_cb);
	ros::Subscriber sub_twist=nh.subscribe("/desired_twist",1,sub_desired_twist_cb);
	
	while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}

	cout << "Qi=" <<pos_i_Robot<<endl;
	
	Vector<> pos_i_DH= yaskawa.joints_Robot2DH(pos_i_Robot);
	
	//Ricavo posa e orientamento.
	T_init = yaskawa.fkine(pos_i_DH);
	
	//Messaggio che pubblicherò sul topic joint_command
	sensor_msgs::JointState outmsg;
    outmsg.position.resize( yaskawa.getNumJoints() );
    outmsg.velocity.resize( yaskawa.getNumJoints() );
    outmsg.effort.resize( yaskawa.getNumJoints() );
    
    outmsg.name.push_back("joint_s");
	outmsg.name.push_back("joint_l");
	outmsg.name.push_back("joint_e");
	outmsg.name.push_back("joint_u");
	outmsg.name.push_back("joint_r");
	outmsg.name.push_back("joint_b");
	outmsg.name.push_back("joint_t");
    
    
    
    //Inizializzazione variabili
    Vector<> qDH_k = pos_i_DH;
    Vector<> qpDH = Zeros(yaskawa.getNumJoints());
    UnitQuaternion oldQ(T_init);
    Vector<6> error = Ones; 
    
	pos=transl(T_init);
	quat=oldQ;
	vel=Zeros(yaskawa.getNumJoints());
	w=Zeros(yaskawa.getNumJoints());
			
	cout << "START" << endl;

    double hz = 1000.0;
    ros::Rate loop_rate(hz);

    double time_now = ros::Time::now().toSec();

    while(ros::ok()){
				
		ros::spinOnce();
		
		        qDH_k = yaskawa.clik(   
                                qDH_k, //<- qDH attuale
                                pos, // <- posizione desiderata
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

        for( int i = 0; i<yaskawa.getNumJoints(); i++ ){
            outmsg.position[i] = qR[i] ;
            outmsg.velocity[i] = qpR[i];
            outmsg.effort[i]=0.0;
        }
        pub_joints.publish(outmsg);

        loop_rate.sleep();

    }
    return 0;
}
