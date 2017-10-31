#include <ros/ros.h>
#include "alireza/Hri.h"
#include "alireza/SpeechFeedback.h"

using namespace std;

int previous = 99;	//added global variable previous of datatype int (avoid passing between functions)

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
//Start
 else if(feedback->content.compare("howfeeling")==0)
  	{
 		previous = 1;	// 1 = previous state set to howfeeling

 		system("aplay ifeelfine.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 //here is where I’d set previous = 0 if this was a why condition
 	}

 else if (feedback->content.compare("doyoulikehere") == 0)
 	{
 		previous = 2;	// 2 = previous state set to doyoulikehere

 		system("aplay ilikeithere.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("doyoulikeme") == 0)
 	{
 		previous = 3;	// 3 = previous state set to doyoulikeme

 		system("aplay ilikeyou.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("doyoulikeyou") == 0)
 	{
 		previous = 4;	// 4 = previous state set to doyoulikeyou

 		system("aplay ilikeme.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("meaningoflife") == 0)
 	{
 		previous = 5;	// 5 = previous state set to meaningoflife

 		system("aplay 42.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("questionoflife") == 0)	// NO ASSIGN PREVIOUS STATE 0 (no sence)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay idontknow.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("followthelaw") == 0)
 	{
 		previous = 6;	// 6 = previous state set to followthelaw

 		system("aplay notallofthem.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("friends") == 0)		// NO ASSIGN PREVIOUS STATE 0 (no sence)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay friends.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("likepepper") == 0)
 	{
 		previous = 7;	// 7 = previous state set to likepepper

 		system("aplay likepepper.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("family") == 0)		// NO ASSIGN PREVIOUS STATE 0 (no sence)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay family.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("creator") == 0)		// NO ASSIGN PREVIOUS STATE 0 (no sence)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay creator.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("myage") == 0)
 	{
 		previous = 8;	// 8 = previous state set to myage

 		system("aplay myage.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("internetaccess") == 0)
 	{
 		previous = 9;	// 9 = previous state set to internetaccess

 		system("aplay cantaccessinternet.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("ipaddress") == 0)
 	{
 		previous = 10;	// 10 = previous state set to ipaddress
 		system("aplay ipaddress.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("johnsnow") == 0)
 	{
 		previous = 99;	// 99 = previous state set to johnsnow
 		system("aplay thatmakesnosense.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("why") == 0)
 	{
 		switch(previous){
 		case 0:
 			system("aplay tryaskingsomethingelse.wav");
 			previous = 0;	//reset
 			break;
 		case 1:		//howfeeling
 			system("aplay justbecause.wav");
 			previous = 0;	//reset
 			break;
 		case 2:		//doyoulikehere
 			system("aplay everyoneisnicehere.wav");
 			previous = 0;	//reset
 			break;
 		case 3:		//doyoulikeme
 			system("aplay funconversations.wav");
 			previous = 0;	//reset
 			break;
 		case 4:		//doyoulikeyou
 			system("aplay itscomplicated.wav");
 			previous = 0;	//reset
 			break;
 		case 5:		//meaningoflife
 			system("aplay deepthoughttoldme.wav");
 			previous = 0;	//reset
 			break;
 		case 6:		//followthelaw
 			system("aplay iamanexception.wav");
 			previous = 0;	//reset
 			break;
 		case 7:		//likepepper
 			system("aplay pepperonlylikesnaoandromeo.wav");
 			previous = 0;	//reset
 			break;
 		case 8:		//myage
 			system("aplay howold.wav");
 			previous = 0;
 			break;
 		case 9:		//internetaccess
 			system("aplay itsdangerous.wav");
 			previous = 0;	//reset
 			break;
 		case 10:	//ipaddress
 			system("aplay notallowed.wav");
 			previous = 0;	//reset
 			break;
 		case 99:	//SPECIAl CASE : MAKES NO SENCE
 			system("aplay thatmakesnosense.wav");
 			previous = 0;	//reset
 			break;
 		default:
 			system("aplay idontunderstand.wav");
 			previous = 0;	//reset
 			break;
 		}
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
