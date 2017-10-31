#include <ros/ros.h>
#include "social_robot/social_robot.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "alireza/Hri.h"
#include "alireza/SpeechFeedback.h"
#include <facedetector/Detection.h>

using namespace std;
 
bool SpeechIsCompleted = false, speechIsReceived = false, eyesAreClosed = false, faceIsDetected = false;
int faceCounter = 0, idleFace = 0, elapsedTime = 0, faceTimeCounter = 0, faceSeqCounter = 0, prevFaceTimeCounter = 0;



class VisualClass
{
 public:
	VisualClass();

 private:
	ros::NodeHandle nh;
	ros::Subscriber feedback_sub;
	ros::Subscriber speech_feedback_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber face_detect_sub;
	SocialRobot social_robot;

	void sfCB(const alireza::SpeechFeedback::ConstPtr &sf);
	void feedbackCB(const alireza::Hri::ConstPtr &feedback);
	void odomCB(const nav_msgs::Odometry::ConstPtr &odom);
	void faceDetectCB(const facedetector::Detection::ConstPtr &detect);

};

VisualClass::VisualClass()
{
 feedback_sub = nh.subscribe<alireza::Hri>("feedback_request",1,&VisualClass::feedbackCB, this);
 speech_feedback_sub = nh.subscribe<alireza::SpeechFeedback>("speech_feedback",1,&VisualClass::sfCB, this);  
 odom_sub = nh.subscribe<nav_msgs::Odometry>("odom",1,&VisualClass::odomCB,this);
 face_detect_sub = nh.subscribe<facedetector::Detection>("/facedetector/faces",1,&VisualClass::faceDetectCB,this);

 social_robot.initialize();
 social_robot.runPeriodically();
 social_robot.shutdown();
}
//Face detection callback
void VisualClass::faceDetectCB(const facedetector::Detection::ConstPtr &detect)
{
	faceTimeCounter = detect->header.stamp.sec;
	/*if (faceTimeCounter == 0)
	{
		faceTimeCounter = detect->header.stamp.sec;
	}
	if (detect->header.stamp.sec-faceTimeCounter == 1)
	{
		faceSeqCounter++;
	}
	if (faceSeqCounter >= 4 && detect->header.stamp.sec-faceTimeCounter == 5)
	{
			faceIsDetected = true;
	}
	else 
	{
		faceTimeCounter = 0;
		faceSeqCounter = 0;
		faceIsDetected = false;
	}*/
  faceIsDetected = true;
} 

/*facial update is taking place within a time-updating function*/
void VisualClass::odomCB(const nav_msgs::Odometry::ConstPtr &odom) 
{
 if (odom->header.stamp.sec - faceTimeCounter >= 3)
 {
	faceIsDetected = false;
 }
 if (speechIsReceived && !SpeechIsCompleted)
 {
	if (faceCounter == idleFace)
	{
		faceCounter = 5;
	}
	else if (faceCounter != idleFace)
	{
		faceCounter++;
	}
 	if (faceCounter > 8)
	{
		faceCounter = 5;
	}
  	social_robot.drawMouth(faceCounter);
 } 
 if (!speechIsReceived || SpeechIsCompleted)
 {
	if (faceIsDetected)
	{
		social_robot.drawMouth(9);
	}
	else
	{
		faceCounter = idleFace;
		social_robot.drawMouth(idleFace);
		speechIsReceived = false;
		SpeechIsCompleted = false;
	}
	
 }
 //EYE Interaction
 if (elapsedTime == 0)
 	{
		elapsedTime = odom->header.stamp.sec;
	}
 if (odom->header.stamp.sec - elapsedTime == 10 && !eyesAreClosed)
 	{
		social_robot.RGBLedsControl(char(0),char(0),char(0),char(0),char(100));
		usleep(100);
  	social_robot.RGBLedsControl(char(1),char(0),char(0),char(0),char(100));
		elapsedTime = odom->header.stamp.sec;
    eyesAreClosed = true;
 	}
  if (odom->header.stamp.sec - elapsedTime == 2 && eyesAreClosed)
 	{
		social_robot.RGBLedsControl(char(0),char(0),char(0),char(100),char(100));
		usleep(100);
  	social_robot.RGBLedsControl(char(1),char(0),char(0),char(100),char(100));
		elapsedTime = odom->header.stamp.sec;
    eyesAreClosed = false;		
 	}
 
 
}
void VisualClass::feedbackCB(const alireza::Hri::ConstPtr &feedback)
{
 speechIsReceived = true;
 if (feedback->feeling.compare("happy") == 0)
 {
		idleFace = 0;
 }
 if (feedback->feeling.compare("sad") == 0)
 {
		idleFace = 1;
 }
 if (feedback->feeling.compare("christmas") == 0)
 {
		idleFace = 0;
		social_robot.Blink(char(0),char(100),char(0),char(10));
 }

 if (feedback->feeling.compare("angry") == 0)
 {
		idleFace = 1;
		social_robot.Blink(char(100),char(0),char(0),char(10));
 }
 faceCounter = idleFace;
}

void VisualClass::sfCB(const alireza::SpeechFeedback::ConstPtr &sf)
{
 SpeechIsCompleted = sf->speechIsCompleted;
}

int main(int argc, char** argv)
{
 ros::init(argc,argv,"visual_feedback");
 VisualClass vc; 
 ros::spin();
 return 0;
}
