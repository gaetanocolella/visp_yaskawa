#include <visp/vpImage.h> 
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpImageSimulator.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/vision/vpPose.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <cv_bridge/cv_bridge.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>
#include <fstream>
#include <iostream>

using namespace std;

int NUM_KEYPOINTS=5; 

ros::Publisher cmdvel;
//ros::Publisher current_pose;
ros::Subscriber depth_sub;
std_msgs::Float64MultiArray msg;
cv_bridge::CvImagePtr cv_ptr;
int DEPTH_HEIGHT, DEPTH_WIDTH;

void readDepth(const sensor_msgs::ImageConstPtr& msg)
{
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
   /* 
    ofstream file;
	 file.open("/home/yaskawa/catkin_visservo/src/visp_yaskawa/objects/OBJ_NAME.txt");
	 
    for (int i =0; i<DEPTH_WIDTH; i++){
    	for (int j =0; j<DEPTH_HEIGHT; j++){
    		file<< (double)cv_ptr->image.at<short int>(cv::Point(i, j))/1000 <<" ";
    		//cout<<temp<<endl;
    		}
    		file<< " \n";
    }

    file.close();
    depth_sub.shutdown();
    cout<<"finish ----------------------------------------"<<endl;*/
}                                        

void setVelocity(const vpColVector &vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = vel[0];
    msg.linear.y = vel[1];
    msg.linear.z = vel[2];
    msg.angular.x = vel[3];
    msg.angular.y = vel[4];
    msg.angular.z = vel[5];
    cmdvel.publish(msg);
}
          
int main(int argc, char **argv) 
{   
  vpROSRobot robot; 
  robot.setCmdVelTopic("/myrobot/cmd_vel");
  robot.init();
  
  ros::NodeHandle n;
  //depth_sub= n.subscribe("/camera/depth/image_rect_raw", 1, readDepth);
  depth_sub= n.subscribe("/camera/aligned_depth_to_color/image_raw", 1, readDepth);
  cmdvel = n.advertise<geometry_msgs::Twist>("/visp_twist", 1);
  //current_pose = n.advertise<std_msgs::Float64MultiArray>("/current_pose", 1);
  vpImage<unsigned char> I;
  
  /*while (ros::ok()){  //decommentare per la scrittura su file della depth map desiderata
  }*/
  
try {
   std::string obj = "";
    
   for (int i = 1; i < argc; i++) {
     if (std::string(argv[i]) == "--obj" && i+1 < argc) {
      obj = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: \n" << argv[0]
                << " [--obj <obj_des.png> <depth_aligned.txt>]";                   
      return 0;
      }
   }
     
  if (obj.empty()) {
  		cout<<"Error object not defined --help"<<endl;
		return 0;
  }

#if defined(VISP_HAVE_OPENCV) && (defined(VISP_HAVE_V4L2) || (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpROSGrabber g; 
  g.setImageTopic("/camera/color/image_raw");
  g.open(I); 
  //vpCameraParameters cam(915.889892578125, 915.9085693359375, 636.5613403320312, 357.21319580078125); 1280x720
  vpCameraParameters cam(610.59326171875, 610.605712890625, 317.7075500488281, 238.1421356201172); //640x480
#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI display;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV display;
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif

#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
    IplImage *cvI = NULL;
#else
    cv::Mat cvI;
#endif

    vpImageConvert::convert(I, cvI);
    vpKltOpencv tracker;
    // Set tracker parameters
    tracker.setMaxFeatures(500);
    tracker.setWindowSize(10);
    tracker.setQuality(0.01);
    tracker.setMinDistance(15);
    tracker.setHarrisFreeParameter(0.04);
    tracker.setBlockSize(9);
    tracker.setUseHarris(1);
    tracker.setPyramidLevels(3);

	 vpImage<unsigned char> Idisp;
	 vpImage<unsigned char> I_static;
	 vpImageIo::read(I_static, "/home/yaskawa/catkin_visservo/src/add/visp_yaskawa/objects/" + obj + ".png");   

	 //[Pref_computation]
	 const std::string detectorName = "ORB";
	 const std::string extractorName = "ORB";
	 const std::string matcherName = "BruteForce-Hamming";
	 vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
	 vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
	 unsigned int nbMatch = keypoint.matchPoint(Idisp);
	 std::cout << "Reference keypoints=" << keypoint.buildReference(I_static) << " , " << nbMatch <<std::endl;
	 Idisp.resize(I.getHeight(), 2 * I.getWidth());
	 Idisp.insert(I_static, vpImagePoint(0, I.getWidth()));
	 Idisp.insert(I, vpImagePoint(0, 0));
	 vpDisplayOpenCV d(Idisp, 0, 0, "Homography from matched keypoints");
	 vpDisplay::display(Idisp);
	 vpDisplay::flush(Idisp);

    //Visual servoing parameters
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.setLambda(0.1);
    bool has_converged = false;
    double convergence_threshold = 0.008;

    
  
	 //Read depth data
  	 float mat[DEPTH_WIDTH][DEPTH_HEIGHT];
	 ifstream rfile("/home/yaskawa/catkin_visservo/src/add/visp_yaskawa/objects/"+ obj + ".txt");
	
	 while(!rfile.eof()){
		 for (int i =0; i<DEPTH_WIDTH; i++){
			for (int j =0; j<DEPTH_HEIGHT; j++){
				rfile >> mat[i][j];
				}
		}
	 }
	rfile.close();
	// visualizza dati
	//cout <<  mat[361][143] << " " << mat[143][361] << " "  << endl;	
			
	nbMatch = keypoint.matchPoint(I);
	std::cout << "Matches=" << nbMatch << std::endl;
	
	//Features initialization
    vpFeaturePoint p_cur[NUM_KEYPOINTS], pd[NUM_KEYPOINTS],p_temp;
    vpFeatureDepth z_cur[NUM_KEYPOINTS], zd[NUM_KEYPOINTS];
    
    for (unsigned int i = 0; i < NUM_KEYPOINTS; i++) {
   		p_cur[i].set_x(0.0);
   		p_cur[i].set_y(0.0);
   		p_cur[i].set_Z(1.0);
   		pd[i].set_x(0.0);
   		pd[i].set_y(0.0);
   		pd[i].set_Z(1.0);
   		
   		z_cur[i].set_x(0.0);
   		z_cur[i].set_y(0.0);
   		z_cur[i].set_Z(1.0);
   		zd[i].set_x(0.0);
   		zd[i].set_y(0.0);
   		zd[i].set_Z(1.0);
      	task.addFeature(p_cur[i], pd[i]);
      	task.addFeature(z_cur[i], zd[i]);
   }
			 
	std::vector<vpImagePoint> iPref, iPcur;
	std::vector<cv::Point2f> feature;
	iPref.resize(NUM_KEYPOINTS);
	iPcur.resize(NUM_KEYPOINTS);
	
	for (unsigned int i = 0; i < NUM_KEYPOINTS; i++){
		keypoint.getMatchedPoints(i, iPref[i], iPcur[i]);
		//feature.push_back(cv::Point2f((float)iPcur[i].get_u(), (float)iPcur[i].get_v()));
		vpFeatureBuilder::create(pd[i], cam, iPref[i]);	
		pd[i].set_Z(mat[(int)iPref[i].get_j()][(int)iPref[i].get_i()]);
		if((mat[(int)iPref[i].get_j()][(int)iPref[i].get_i()]) != 0.0){
				zd[i].buildFrom(pd[i].get_x(), pd[i].get_y(), (mat[(int)iPref[i].get_j()][(int)iPref[i].get_i()]), 0);
		}
		else{
			   zd[i].buildFrom(pd[i].get_x(), pd[i].get_y(), 1.0, 0);
			 }
	}
	
	double i_mean=0.0, j_mean=0.0;
	for (unsigned int i = 0; i < NUM_KEYPOINTS; i++) {
		i_mean = i_mean + iPcur[i].get_i();
		j_mean = j_mean + iPcur[i].get_j();
	}
	i_mean=i_mean/NUM_KEYPOINTS;
	j_mean=j_mean/NUM_KEYPOINTS;
	cout<<"Media: "<<i_mean<<" "<<j_mean<<endl;
	
	vpImagePoint mean(i_mean, j_mean);
	
	vpImagePoint iPref_temp, iPcur_temp;

	int count=0;
	
	for (unsigned int i = 0; i < nbMatch; i++){

		keypoint.getMatchedPoints(i, iPref_temp, iPcur_temp);		
		vpFeatureBuilder::create(p_temp, cam, iPcur_temp);		
		p_temp.set_Z((double)cv_ptr->image.at<short int>(cv::Point((int)iPcur_temp.get_j(), (int)iPcur_temp.get_i()))/1000);	
		
		if(p_temp.get_x()<0.10 && p_temp.get_y()<0.10 && p_temp.get_Z()<0.60){
		iPref[count] = iPref_temp;
		iPcur[count] = iPcur_temp;
		cout<<"p_temp.get_Z(): "<<i<<" "<<p_temp.get_Z()<<endl;
		feature.push_back(cv::Point2f((float)iPcur[i].get_u(), (float)iPcur[i].get_v()));
      vpFeatureBuilder::create(pd[count], cam, iPref[i]);	
		pd[count].set_Z(mat[(int)iPref[i].get_j()][(int)iPref[i].get_i()]);
		if((mat[(int)iPref[i].get_j()][(int)iPref[i].get_i()]) != 0.0){
				zd[count].buildFrom(pd[count].get_x(), pd[count].get_y(), (mat[(int)iPref[i].get_j()][(int)iPref[i].get_i()]), 0);
		}
		else{
			   zd[count].buildFrom(pd[count].get_x(), pd[count].get_y(), 1.0, 0);
			 }
		count++;
		}
		if(count == NUM_KEYPOINTS)break;
	}
	
	
	 
	tracker.initTracking(cvI, feature);
	std::vector<cv::Point2f> feature_cur;
	std::vector<long> id_feat;
	
	while (!has_converged) {	   
		   g.acquire(I);
		   Idisp.insert(I, vpImagePoint(0, 0));
		   vpDisplay::display(Idisp);
		   vpImageConvert::convert(I, cvI);
		   		   		   
		   		   		   
		   		   		   
		   for (unsigned int i = 0; i < NUM_KEYPOINTS; i++) {
		   		vpDisplay::displayCross(Idisp, iPref[i]+vpImagePoint(0, I.getWidth()), 12, vpColor::green);
               //vpDisplay::displayCross(Idisp, iPcur[i], 12, vpColor::blue);
         }
         tracker.track(cvI);
		   feature_cur = tracker.getFeatures();
	   
		   //cout << "tracked_feat: "<< feature_cur.size() << endl;    
      	id_feat = tracker.getFeaturesId();
      	
      	/*for(unsigned int i = 0; i < id_feat.size(); i++)
      			   cout << id_feat[i] << " ";    
      	cout << endl;*/
      	
   
 	
		   for (unsigned int i = 0; i < NUM_KEYPOINTS; i++) {
				p_cur[i].set_x(0.0);
				p_cur[i].set_y(0.0);
				p_cur[i].set_Z(1.0);
				pd[i].set_x(0.0);
				pd[i].set_y(0.0);
				pd[i].set_Z(1.0);
				
				z_cur[i].set_x(0.0);
				z_cur[i].set_y(0.0);
				z_cur[i].set_Z(1.0);
				z_cur[i].set_LogZoverZstar(0.0);
				zd[i].set_x(0.0);
				zd[i].set_y(0.0);
				zd[i].set_Z(1.0);
				zd[i].set_LogZoverZstar(0.0);
		 	 }
      	
      	for(unsigned int i = 0; i < id_feat.size(); i++){      	
      		for(int j=0; j<NUM_KEYPOINTS; j++){
      	   	if(j == id_feat[i]){      	   	   
						vpImagePoint ip_feat_cur(feature_cur[i].y, feature_cur[i].x);
						vpFeatureBuilder::create(p_cur[j], cam, ip_feat_cur);	
						p_cur[j].set_Z((double)cv_ptr->image.at<short int>(cv::Point((int)ip_feat_cur.get_j(), (int)ip_feat_cur.get_i()))/1000);
	
						vpFeatureBuilder::create(pd[j], cam, iPref[j]);	
						pd[j].set_Z(mat[(int)iPref[j].get_j()][(int)iPref[j].get_i()]);
						
						if(((double)cv_ptr->image.at<short int>(cv::Point((int)ip_feat_cur.get_j(), (int)ip_feat_cur.get_i()))/1000) == 0.0 || (mat[(int)iPref[j].get_j()][(int)iPref[j].get_i()]) == 0.0){
								pd[j].set_Z(1.0);
								p_cur[j].set_Z(1.0);
								
								z_cur[j].set_Z(1.0);
								z_cur[j].set_LogZoverZstar(0.0);
								zd[j].set_Z(1.0);
								zd[j].set_LogZoverZstar(0.0);
						}
						else{	
								z_cur[j].buildFrom(p_cur[j].get_x(), p_cur[j].get_y(), ((double)cv_ptr->image.at<short int>(cv::Point((int)ip_feat_cur.get_j(), (int)ip_feat_cur.get_i()))/1000), log(((double)cv_ptr->image.at<short int>(cv::Point((int)ip_feat_cur.get_j(), (int)ip_feat_cur.get_i()))/1000)/(mat[(int)iPref[j].get_j()][(int)iPref[j].get_i()])));
					   		zd[j].buildFrom(pd[j].get_x(), pd[j].get_y(), (mat[(int)iPref[j].get_j()][(int)iPref[j].get_i()]), 0);
						}						
						break;
					}
				}			
		   }

			/*for (unsigned int i = 0; i < NUM_KEYPOINTS; i++) {
				cout<<"z_d: "<<zd[i].get_x()<<" "<<zd[i].get_y()<<" "<<zd[i].get_Z()<<" "<<zd[i].get_LogZoverZstar()<<endl;
		   	cout<<"z_cur: "<<z_cur[i].get_x()<<" "<<z_cur[i].get_y()<<" "<<z_cur[i].get_Z()<<" "<<z_cur[i].get_LogZoverZstar()<<endl;		   			   		
		   	cout<<"p_d: "<<pd[i].get_x()<<" "<<pd[i].get_y()<<" "<<pd[i].get_Z()<<endl;
		   	cout<<"p_cur: "<<p_cur[i].get_x()<<" "<<p_cur[i].get_y()<<" "<<p_cur[i].get_Z()<<endl;
		   }	*/	   
		   
		  	tracker.display(Idisp, vpColor::red);
		  	
		  	
		  	vpDisplay::displayCross(Idisp, mean, 50, vpColor::white);
		  	
		   //Compute control low 	
		  	vpColVector v = task.computeControlLaw();
		   vpColVector e = task.getError();
		   double error = e.sumSquare();
		   cout<<"Errore: "<<e.size()<<" "<<error<<endl;   
		   //cout<<"e:"<<" "<<e.t()<<endl;
	  
		   //stampa v:
		 	/*std::cout << "v:" << std::endl;
		 	for (unsigned int i = 0; i < v.size(); i++) {
		   	std::cout << v[i] << std::endl;
		 	}*/
		 	
		 	if (error < convergence_threshold) {
		   	has_converged = true;
		   	std::cout << "Servo task has converged" << "\n";
		   	vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
		   }

			robot.setVelocity(vpRobot::CAMERA_FRAME, v);
	  		msg.data.resize(7);
	  		setVelocity(v); 		
		   vpDisplay::flush(Idisp);
		     
		 	if (vpDisplay::getClick(Idisp, false)){
		   	break;
  			}
  			
  		   //cout<<"-----------------------------------------------------------------"<<endl;
  		}
		
	   vpColVector v0(6);
	  
	   for(int i =0; i<6; i++){
	  		v0[i]=0;
	   }
	  
	   for(int i =0; i<10; i++){
	  	  setVelocity(v0);
	  	  cout<<"Task terminato!"<<endl;
	   }		
		
	   task.kill();
	   sleep(3);
	 } 
	 catch (const vpException &e) {
		 std::cout << "Catch an exception: " << e << std::endl;
		 vpColVector v0(6);	  
	 	for(int i =0; i<6; i++){
	  	  v0[i]=0;
	  	}	  		
		for(int i =0; i<10; i++){
	  			setVelocity(v0);
	  			cout<<"Task terminato!"<<endl;
	  	}	
	 }	 
#endif
return 0;
}
  
  
  
  
  
  
  
  
  
  
  
  
  
  

