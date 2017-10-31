#include <ros/ros.h>
#include "alireza/Hri.h"
#include "alireza/SpeechFeedback.h"

using namespace std;

alireza::SpeechFeedback SF;

class PreacherClass
{
 public:
 	PreacherClass();
 private:
	ros::NodeHandle nh;
	ros::Subscriber feedback_sub;
	ros::Publisher speech_feedback_pub;
	void SpeechCB(const alireza::Hri::ConstPtr &feedback);
};

PreacherClass::PreacherClass()
{
	feedback_sub = nh.subscribe<alireza::Hri>("feedback_request",1,&PreacherClass::SpeechCB,this);
	speech_feedback_pub = nh.advertise<alireza::SpeechFeedback>("speech_feedback",5);
}

void PreacherClass::SpeechCB(const alireza::Hri::ConstPtr &feedback)
{
 
 if(feedback->content.compare("welcome")==0)
 {
	system("aplay welcome.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("arewegoingsomewhere")==0)
 {
	system("aplay arewegoingsomewhere.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("appriciate")==0)
 {
	system("aplay thanks.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("iamhungrypleasepowermeup")==0)
 {
	system("aplay iamhungrypleasepowermeup.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("becarefuliamfragile")==0)
 {
	system("aplay becarefuliamfragile.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("ok")==0)
 {
	system("aplay ok.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("ididthatalready")==0)
 {
	system("aplay ididthatalready.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("ouch")==0)
 {
	system("aplay ouch.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("youneedtounplugmefirst")==0)
 {
	system("aplay youneedtounplugmefirst.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("yourwelcome")==0)
 {
	system("aplay yourwelcome.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("cheese")==0)
 {
	system("aplay cheese.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("seeyoulater")==0)
 {
	system("aplay seeyoulater.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("hello")==0)
 {
	system("aplay hello.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("yes")==0)
 {
	system("aplay yes.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("mynameisbetty")==0)
 {
	system("aplay mynameisbetty.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("icannt")==0)
 {
	system("aplay icannt.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("waiticannotseeyou")==0)
 {
	system("aplay waiticannotseeyou.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("okleadtheway")==0)
 {
	system("aplay okleadtheway.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("areyouready")==0)
 {
	system("aplay areyouready.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("click")==0)
 {
	system("aplay click.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("no")==0)
 {
	system("aplay no.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }

 else if(feedback->content.compare("two")==0)
 {
	system("aplay two.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;	
	speech_feedback_pub.publish(SF);
 }
 
 else //if the speech does not exist, we just pretend it's done for idle face
 {
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
}

int main (int argc, char** argv)
{
 ros::init(argc,argv,"preacher");
 PreacherClass pc;
 ros::spin();
 return 0;
}
