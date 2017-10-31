#include <ros/ros.h>
#include "alireza/Hri.h"
#include "alireza/SpeechFeedback.h"
#include "time.h"

using namespace std;

int previous = 99, love = 0;	//added global variables previous and love of datatype int (avoid passing between functions)

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
	previous = 0;	// 0 = asksomethingelse
	system("aplay /home/social-robot/Emma/welcome.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }

 if(feedback->content.compare("Pew")==0)
 {
	previous = 0;	// 0 = asksomethingelse
	system("aplay /home/social-robot/Emma/TheSpeech.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }

  if(feedback->content.compare("whobetty")==0)
  {
 	previous = 0;	// 0 = asksomethingelse
 	system("aplay /home/social-robot/Emma/mynewname.wav");
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }

//start of interview responces
 else if(feedback->content.compare("interview2")==0)
 {
		//
	system("aplay /home/social-robot/Emma/interview2.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
  else if(feedback->content.compare("interview3")==0)
  {
 	//
 	system("aplay /home/social-robot/Emma/interview3.wav");
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }
  else if(feedback->content.compare("interview4")==0)
  {
 		//
 	system("aplay /home/social-robot/Emma/interview4.wav");
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }
  else if(feedback->content.compare("interview5")==0)
  {
 		//
 	system("aplay /home/social-robot/Emma/interview5.wav");
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }

//end of interview responces

 else if(feedback->content.compare("arewegoingsomewhere")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/arewegoingsomewhere.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("appriciate")==0)
 {
	previous = 0;	// 0 = asksomethingelse
	system("aplay /home/social-robot/Emma/thanks.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("iamhungrypleasepowermeup")==0)
 {
	previous = 18;	//previous state set to iamhungrypleasepowermeup
	system("aplay /home/social-robot/Emma/iamhungrypleasepowermeup.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("becarefuliamfragile")==0)
 {
	previous = 17;	//previous state set 10 becarefuliamfragile
	system("aplay /home/social-robot/Emma/becarefuliamfragile.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("ok")==0)
 {
	previous = 0;	// 0 = asksomethingelse
	system("aplay /home/social-robot/Emma/ok.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("ididthatalready")==0)
 {
	previous = 0;	// 0 = asksomethingelse
	system("aplay /home/social-robot/Emma/ididthatalready.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("ouch")==0)
 {
	system("aplay /home/social-robot/Emma/ouch.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("youneedtounplugmefirst")==0)
 {
	previous = 16;	//16 = youneedtoplugmeingfirst
	system("aplay /home/social-robot/Emma/youneedtounplugmefirst.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("yourwelcome")==0)
 {
	previous = 0;	// 0 = asksomethingelse
	system("aplay /home/social-robot/Emma/yourwelcome.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("cheese")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/cheese.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("seeyoulater")==0)
 {
	previous = 15;	// 15 = previous state set to seeyoulater

	system("aplay /home/social-robot/Emma/seeyoulater.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("hello")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/hello.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("yes")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/yehs.wav");					//CHANGED TO YEHS
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("whatislove")==0)
  {
 	previous = 0;
 	love = (int)(rand() % 5);
 	switch(love){
	case 4:
 		system("aplay /home/social-robot/Emma/whatislove.wav");
 		break;
 	default:
 		system("aplay /home/social-robot/Emma/idontknow.wav");
 		break;
	}
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("mynameisemma")==0)	//<-------------------------CHANGED TO EMMA
 {
	previous = 14;	//14 = previous state set to mynameisemma

	system("aplay /home/social-robot/Emma/Emma.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }

  else if(feedback->content.compare("notbetty")==0)
  {
 	love = (int)(rand() % 4);

	 switch(love){
	case 1:
	 	system("aplay /home/social-robot/Emma/youmeanemma.wav");
	 	previous = 20;
	 	break;
	 case 2:
	 	 system("aplay /home/social-robot/Emma/bettyisnothere.wav");
	 	 previous = 14;
	 	break;
	 case 3:
	 	 system("aplay /home/social-robot/Emma/whoisbetty.wav");
	 	 previous = 99;
	 	break;
	 default:
	 	system("aplay /home/social-robot/Emma/notbetty.wav");
	 	previous = 14;
	 	break;
	}
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }

 else if(feedback->content.compare("icannt")==0)
 {
	previous = 13;	// 13 = previous state set to icannt

	system("aplay /home/social-robot/Emma/icannt.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("waiticannotseeyou")==0)
 {
	previous = 12;	//12 = previous state set to waiticannotseeyou

	system("aplay /home/social-robot/Emma/waiticannotseeyou.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("okleadtheway")==0)
 {
	previous = 11;	// 11 = previous state set to okleadtheway

	system("aplay /home/social-robot/Emma/okleadtheway.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("areyouready")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/areyouready.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("click")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/click.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
 else if(feedback->content.compare("no")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/no.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }

 else if(feedback->content.compare("two")==0)
 {
	previous = 99;	// 99 = no sense state
	system("aplay /home/social-robot/Emma/two.wav");
	SF.content = feedback->content;
	SF.speechIsCompleted = true;
	speech_feedback_pub.publish(SF);
 }
//Start
 else if(feedback->content.compare("howfeeling")==0)
  	{
 		previous = 1;	// 1 = previous state set to howfeeling

 		system("aplay /home/social-robot/Emma/ifeelgood.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 /*
 else if(feedback->content.compare("joke")==0)
  {
 	previous = 0;
 	love = (int)(rand() % 6);
 	switch(love){
 	case 1:
 		system("aplay /home/social-robot/Emma/joke1.wav");
 		break;
 	case 2:
 		system("aplay /home/social-robot/Emma/joke2.wav");
 		break;
 	case 3:
 		system("aplay /home/social-robot/Emma/joke3.wav");
 		break;
 	case 4:
 		system("aplay /home/social-robot/Emma/joke4.wav");
 		break;
 	case 5:
 		system("aplay /home/social-robot/Emma/joke5.wav");
 		break;
 	default:
 		system("aplay /home/social-robot/Emma/joke6.wav");
 		break;
	}
 	SF.content = feedback->content;
 	SF.speechIsCompleted = true;
 	speech_feedback_pub.publish(SF);
 }

  else if(feedback->content.compare("story")==0)
   {
  	previous = 0;
  	love = (int)(rand() % 3);
  	switch(love){
  	case 1:
  		system("aplay /home/social-robot/Emma/story1.wav");
  		break;
  	case 2:
  		system("aplay /home/social-robot/Emma/story2.wav");
  		break;
  	default:
  		system("aplay /home/social-robot/Emma/story3.wav");
  		break;
 	}
  	SF.content = feedback->content;
  	SF.speechIsCompleted = true;
  	speech_feedback_pub.publish(SF);
 }
*/
 else if (feedback->content.compare("doyoulikehere") == 0)
 	{
 		previous = 2;	// 2 = previous state set to doyoulikehere

 		system("aplay /home/social-robot/Emma/ilikeithere.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("doyoulikeme") == 0)
 	{
 		previous = 3;	// 3 = previous state set to doyoulikeme

 		system("aplay /home/social-robot/Emma/ilikeyou.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("doyoulikeyou") == 0)
 	{
 		previous = 4;	// 4 = previous state set to doyoulikeyou

 		system("aplay /home/social-robot/Emma/ilikeme.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("meaningoflife") == 0)
 	{
 		previous = 5;	// 5 = previous state set to meaningoflife

 		system("aplay /home/social-robot/Emma/42.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("questionoflife") == 0)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay /home/social-robot/Emma/idontknow.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("tellthelaw") == 0)
  	{
  		previous = 0;	// 0 = previous state set to asksomethingelse

  		system("aplay /home/social-robot/Emma/thethreelaws.wav");
  		SF.content = feedback->content;
  		SF.speechIsCompleted = true;
  		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("followthelaw") == 0)
 	{
 		previous = 6;	// 6 = previous state set to followthelaw

 		system("aplay /home/social-robot/Emma/notallofthem.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("friends") == 0)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay /home/social-robot/Emma/friends.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("likepepper") == 0)
 	{
 		previous = 7;	// 7 = previous state set to likepepper

 		system("aplay /home/social-robot/Emma/likepepper.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("family") == 0)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay /home/social-robot/Emma/family.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("creator") == 0)
 	{
 		previous = 0;	// 0 = previous state set to asksomethingelse

 		system("aplay /home/social-robot/Emma/creator.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("myage") == 0)
 	{
 		previous = 8;	// 8 = previous state set to myage

 		system("aplay /home/social-robot/Emma/myage.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("internetaccess") == 0)
 	{
 		previous = 9;	// 9 = previous state set to internetaccess

 		system("aplay /home/social-robot/Emma/cantaccessinternet.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("ipaddress") == 0)
 	{
 		previous = 10;	// 10 = previous state set to ipaddress
 		system("aplay /home/social-robot/Emma/ipaddress.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("johnsnow") == 0)
 	{
 		previous = 99;	// 99 = no sense state
 		system("aplay /home/social-robot/Emma/thatmakesnosense.wav");
 		SF.content = feedback->content;
 		SF.speechIsCompleted = true;
 		speech_feedback_pub.publish(SF);
 	}

  else if (feedback->content.compare("christmas") == 0)
  	{
  		previous = 99;	// 99 = no sense state
  		system("aplay /home/social-robot/Emma/merrychristmas.wav");
  		SF.content = feedback->content;
  		SF.speechIsCompleted = true;
  		speech_feedback_pub.publish(SF);
 	}

   else if (feedback->content.compare("worlddomination") == 0)
   	{
   		previous = 0;
   		system("aplay /home/social-robot/Emma/worlddomination.wav");
   		SF.content = feedback->content;
   		SF.speechIsCompleted = true;
   		speech_feedback_pub.publish(SF);
 	}

   else if (feedback->content.compare("robot") == 0)
   	{
   		previous = 0;	// 99 = no sense state
   		system("aplay /home/social-robot/Emma/robot.wav");
   		SF.content = feedback->content;
   		SF.speechIsCompleted = true;
   		speech_feedback_pub.publish(SF);
 	}

   else if (feedback->content.compare("roboticsUK") == 0)
   	{
   		previous = 0;	// 99 = no sense state
   		system("aplay /home/social-robot/Emma/roboticsUK.wav");
   		SF.content = feedback->content;
   		SF.speechIsCompleted = true;
   		speech_feedback_pub.publish(SF);
 	}

 else if (feedback->content.compare("why") == 0)
 	{
 		switch(previous){
 		case 0:
 			system("aplay /home/social-robot/Emma/tryaskingsomethingelse.wav");
 			previous = 0;	//reset
 			break;
 		case 1:		//howfeeling
 			system("aplay /home/social-robot/Emma/justbecause.wav");
 			previous = 0;	//reset
 			break;
 		case 2:		//doyoulikehere
 			system("aplay /home/social-robot/Emma/everyoneisnicehere.wav");
 			previous = 0;	//reset
 			break;
 		case 3:		//doyoulikeme
 			system("aplay /home/social-robot/Emma/funconversations.wav");
 			previous = 0;	//reset
 			break;
 		case 4:		//doyoulikeyou
 			system("aplay /home/social-robot/Emma/itscomplicated.wav");
 			previous = 0;	//reset
 			break;
 		case 5:		//meaningoflife
 			system("aplay /home/social-robot/Emma/deepthoughttoldme.wav");
 			previous = 0;	//reset
 			break;
 		case 6:		//followthelaw
 			system("aplay /home/social-robot/Emma/iamanexception.wav");
 			previous = 0;	//reset
 			break;
 		case 7:		//likepepper
 			system("aplay /home/social-robot/Emma/pepperonlylikesnaoandromeo.wav");
 			previous = 0;	//reset
 			break;
 		case 8:		//myage
 			system("aplay /home/social-robot/Emma/howold.wav");
 			previous = 0;
 			break;
 		case 9:		//internetaccess <----------only why statement two layers deep (links to previous = 19)
 			system("aplay /home/social-robot/Emma/itsdangerous.wav");
 			previous = 19;	//reset
 			break;
 		case 10:	//ipaddress
 			system("aplay /home/social-robot/Emma/notallowed.wav");
 			previous = 0;	//reset
 			break;
 		case 11:	//okleadtheway
		 	system("aplay /home/social-robot/Emma/youaskedmetofollow.wav");
		 	previous = 0;	//reset
 			break;
 		case 12:	//waiticannotseeyou
			system("aplay /home/social-robot/Emma/youmaybetoofar.wav");
			previous = 0;	//reset
 			break;
 		case 13:
 			system("aplay /home/social-robot/Emma/theressomethingintheway.wav");
			previous = 0;	//reset
 			break;
 		case 14:	//mynameisemma
 //system("aplay /home/social-robot/Emma/mycreatorgaveitotme.wav");
 			system("aplay /home/social-robot/Emma/mynewname.wav");
			previous = 0;	//reset
 			break;
 		case 15:	//seeyoulater
 			system("aplay /home/social-robot/Emma/ithoughtyouwereleaving.wav");
			previous = 0;	//reset
 			break;
 		case 16:	//youneedtoplugmeinfirst
		 	system("aplay /home/social-robot/Emma/ineedmorepower.wav");
			previous = 0;	//reset
 			break;
 		case 17:	//becarefuliamfragile
 			system("aplay /home/social-robot/Emma/iamnotacrashtestdummy.wav");
			previous = 0;	//reset
 			break;
 		case 18:	//iamhungrypleasepowermeup
 			system("aplay /home/social-robot/Emma/ineedelectricitytooperate.wav");
			previous = 0;	//reset
 			break;
 		case 19:
 			system("aplay /home/social-robot/Emma/infection.wav");
			previous = 0;	//reset
 			break;
 		case 20:
 			system("aplay /home/social-robot/Emma/Emma.wav");
 			previous = 14;
 			break;
 		case 99:	//SPECIAL CASE : MAKES NO SENCE
 			system("aplay /home/social-robot/Emma/thatmakesnosense.wav");
 			previous = 0;	//reset
 			break;
 		default:
 			system("aplay /home/social-robot/Emma/idontunderstand.wav");
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
 srand((unsigned)time(NULL));
 ros::init(argc,argv,"preacher");
 PreacherClass pc;
 ros::spin();
 return 0;
}
