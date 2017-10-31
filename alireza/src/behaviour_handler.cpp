#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <social_robot/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <social_robot/BumpersState.h>
#include <social_robot/RollPitch.h>
#include <face_recognition/recperson.h>
#include <face_recognition/orderdone.h>
#include <face_recognition/FRClientGoal.h>
#include <anthony/face.h>
#include <anthony/touch_cmd.h>
#include <wifi/wifi_cmd.h>
#include "social_robot/social_robot.h"
#include "alireza/Hri.h"
#include <ros/callback_queue.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>

#define batteryThreshold 12.0
#define XRES 640

using namespace std;

geometry_msgs::Twist vel;
alireza::Hri speech_cmd;
bool messageIsSent = false;
int B0PrevValue = 0;
int B1PrevValue = 0;
int B2PrevValue = 0;
int convo = 0; // Variable to cycle through responses
bool make8 = false;
bool youAreTooClose = false, warningIsSent = false, weAreMoving = false, bumperMessageSent = false, plugIsAttached = true, roadAheadIsBlocked = false;
float pcBatteryState = 0.0, elecBatteryState = 0.0, motorBatteryState = 0.0, prevCableState = 0.0,linVel = 0.0, angVel = 0.0;
int prevroll = 0, prevpitch = 0; //Will be used to detect bump
bool bumpsent = false, bumpstart = false;
int last_order = 0; //Last finished order
string lastrecperson = "";
bool deleting = false, follow = false, touch_enable = false;

class BettyClass
{
 public:
	BettyClass();
	void recOrder(int order_id, string order_argument);
	void waitOrder(int order_id);

 private:
	ros::NodeHandle nh;
	ros::Publisher vel_pub;
	ros::Publisher speech_request_pub;
	ros::Publisher face_rec_order_pub;
	ros::Subscriber joy_sub;
	ros::Subscriber battery_state_sub;
	ros::Subscriber	laser_sub;
	ros::Subscriber	voice_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber bumper_sub;
	ros::Subscriber rollpitch_sub; //Roll and pitch value used for bump
	ros::Subscriber recperson_sub; //Recognized person
	ros::Subscriber orderdone_sub; //Last order done by face rec
	ros::Subscriber detperson_sub; //Detected person for follow
	ros::Subscriber touch_cmd_sub; //Touch screen commands
	ros::Subscriber wifi_sub; //Wifi commands

	void joyCB(const sensor_msgs::Joy::ConstPtr &joy);
	void batteryCB(const social_robot::BatteryState::ConstPtr &batteryState);
	void laserCB(const sensor_msgs::LaserScan::ConstPtr &laser);
	void voiceCB(const std_msgs::String::ConstPtr &cmd);
	void odomCB(const nav_msgs::Odometry::ConstPtr &odom);
	void rollpitchCB(const social_robot::RollPitch::ConstPtr &rollpitch);
	void recpersonCB(const face_recognition::recperson::ConstPtr &recperson);
	void orderdoneCB(const face_recognition::orderdone::ConstPtr &orderdone);
	void bumperCB(const social_robot::BumpersState::ConstPtr &bumper);
	void detpersonCB(const anthony::face::ConstPtr &detperson);
	void touchCmdCB(const anthony::touch_cmd::ConstPtr &touchCmd);
	void wifiCmdCB(const wifi::wifi_cmd::ConstPtr &wifiCmd);
	bool regNewPerson(std::string filename, std::string name);
	bool deletePerson(std::string filename, std::string tempname, std::string name);
	bool isSomeoneRec(float n);

	void setSpeed(float linearSpeed, float angularSpeed)
	{
		geometry_msgs::Twist vel_;
		vel_.linear.x = linearSpeed;
		vel_.angular.z = angularSpeed;
		vel_pub.publish(vel_);
	}
};

BettyClass::BettyClass()
{
	face_rec_order_pub = nh.advertise<face_recognition::FRClientGoal>("/fr_order",5);
	speech_request_pub = nh.advertise<alireza::Hri>("feedback_request",1);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy",10,&BettyClass::joyCB,this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",4);
  battery_state_sub = nh.subscribe<social_robot::BatteryState>("/battery_state",1,&BettyClass::batteryCB,this);
	laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,&BettyClass::laserCB,this);
	voice_sub = nh.subscribe<std_msgs::String>("/recognizer/output",5,&BettyClass::voiceCB,this);
	//bumper_sub = nh.subscribe<social_robot::BumpersState>("/bumpers_state",1, &BettyClass::bumperCB,this); //deactivated for now
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom",5,&BettyClass::odomCB,this);
	rollpitch_sub = nh.subscribe<social_robot::RollPitch>("/roll_pitch",5,&BettyClass::rollpitchCB,this); //deactivated for now
	recperson_sub = nh.subscribe<face_recognition::recperson>("/recperson",5,&BettyClass::recpersonCB,this);
	orderdone_sub = nh.subscribe<face_recognition::orderdone>("/orderdone",5,&BettyClass::orderdoneCB,this);
	detperson_sub = nh.subscribe<anthony::face>("/singleface",5,&BettyClass::detpersonCB,this);
	touch_cmd_sub = nh.subscribe<anthony::touch_cmd>("/touch_cmd",5,&BettyClass::touchCmdCB,this);
	wifi_sub = nh.subscribe<wifi::wifi_cmd>("/wifi_cmd",5,&BettyClass::wifiCmdCB,this);
}

//Generate the message to command the face_recognition nodes
//order_id value define type of order
//0:recognize once, 1:recognize continuously, 2:add new face, 3:retrain database, 4:quit, 5:delete face
//Only order 3 and 5 need order_argument specific values
void BettyClass::recOrder(int order_id, string order_argument)
{
	if(face_rec_order_pub.getNumSubscribers() >= 1){
		face_recognition::FRClientGoal startrec;
 		startrec.order_id = order_id;
		startrec.order_argument = order_argument;
 		face_rec_order_pub.publish(startrec);
	}
}

void BettyClass::bumperCB(const social_robot::BumpersState::ConstPtr &bumper)
{
	if (bumper->left_bump  == true || bumper->right_bump == true || bumper->front_bump == true || bumper->rear_bump == true)
	{
		if (!bumpsent)
		{
			speech_cmd.content = "ouch"; // ouch
			speech_cmd.feeling = "sad";
			speech_request_pub.publish(speech_cmd);
			bumperMessageSent = true;
		}
	}
	else
	{
			bumpsent = false;
	}
}


//Test for n seconds if someone is recognized
bool BettyClass::isSomeoneRec(float n)
{
	boost::shared_ptr<face_recognition::recperson const> test;
	test = ros::topic::waitForMessage<face_recognition::recperson>("/recperson",nh,(ros::Duration)n); //n seconds test

	if (test == NULL) //Nobody recognized
		return false;
	return true;
}

//Add new person to specified file and give orders to database
bool BettyClass::regNewPerson(std::string filename, std::string name)
{
	fstream file;
	file.open(filename.c_str(),std::fstream::in|std::fstream::out|std::fstream::app);

	if(file.is_open()){
		int count = 0;
		std::string buf;
		while(getline(file, buf)) //How many persons in the file
			count++;
		file.clear();
		stringstream cname;
		cname << count << "_" << name;
		file << cname.str() << endl; //add the new person
		file.close();

		recOrder(2,cname.str()); //Register the person in the face_recognition database
		ros::topic::waitForMessage<face_recognition::orderdone>("/orderdone",nh,(ros::Duration)10.0);
		recOrder(3,"none"); //Update database
		ros::topic::waitForMessage<face_recognition::orderdone>("/orderdone",nh,(ros::Duration)1.0);
		recOrder(1,"none"); //Scan again
	}

	else{
		ROS_WARN("Could not open bad persons file");
		return false;
	}

	return true;
}

//Delete person from specified file and give orders to database
bool BettyClass::deletePerson(std::string filename, std::string tempname, std::string name)
{
	fstream file;
	file.open(filename.c_str(),std::fstream::in|std::fstream::out|std::fstream::app);

	string buf;
	if(file.is_open()){
		ofstream tempfile;
		tempfile.open(tempname.c_str());
		if(tempfile.is_open()){
			while(getline(file,buf))
				if(buf.find(name) == std::string::npos) //Copy only lines to keep, name is passively deleted this way
					tempfile << buf << endl;
			tempfile.close();
		}

		else{
			ROS_WARN("Could not open temporary file.");
			return false;
		}
		file.close();

		recOrder(5,name); //Delete the person
		ros::topic::waitForMessage<face_recognition::orderdone>("/orderdone",nh,(ros::Duration)3.0);
		recOrder(3,"none"); //Update database
		ros::topic::waitForMessage<face_recognition::orderdone>("/orderdone",nh,(ros::Duration)1.0);
		ROS_INFO("%s successfully deleted",name.c_str());
		recOrder(1,"none"); //Scan again
	}

	else{
		ROS_WARN("Could not open file");
		return false;
	}
	std::remove(filename.c_str());
	std::rename(tempname.c_str(),filename.c_str()); //replace file with the one not containing the deleted person
	return true;
}

//Check if betty received a bump
void BettyClass::rollpitchCB(const social_robot::RollPitch::ConstPtr &rollpitch)
{
	if(weAreMoving == false && bumpsent == false && follow == false) //Not considered if the robot is moving or following
	{
		if ((pow(prevroll - rollpitch->roll,2)+pow(prevpitch - rollpitch->pitch,2)) > 50) //Abritrary value to tune
		{
			if(bumpstart == false) //bump init for prevpitch&roll to get current value
				bumpstart = true; //without this flag the robot always start up with a bump trigger although none happened
			else
			{
				speech_cmd.content = "ouch"; //ouch
				speech_cmd.feeling = "sad";
				speech_request_pub.publish(speech_cmd);
				bumpsent = true; //flag so bump is detected only once
				if(face_rec_order_pub.getNumSubscribers() >= 1) //Deactivated is face recognition is not active
					if(((prevroll - rollpitch->roll) < -10) && !isSomeoneRec(2.0f)) //Bump from front from a new person
						regNewPerson(ros::package::getPath("alireza") + "/data/badpersons.txt","badperson");
			}
		}
	}

	else if(prevroll==rollpitch->roll && prevpitch==rollpitch->pitch) //We make sure the bump is over (to notice it once)
		bumpsent = false;

	prevroll = rollpitch->roll; //Update values for next analysis
	prevpitch = rollpitch->pitch;
}

//A person have been recognized
//The "badperson" word is used when the robot registers someone who kicked it. This way it could
//register persons with others name for alternative applications (recognizing someome friendly, etc)
void BettyClass::recpersonCB(const face_recognition::recperson::ConstPtr &recperson)
{
	lastrecperson = recperson->name; //Can be useful for later applications (here used for apologize function)

	if(recperson->name.find("badperson")!=string::npos && !deleting) //The detected face is someone who kicked the robot and no deleting is on going
	{
		speech_cmd.content = "-";
		speech_cmd.feeling = "angry";
		speech_request_pub.publish(speech_cmd);
	}
}

//An order is finished from face recognizer
void BettyClass::orderdoneCB(const face_recognition::orderdone::ConstPtr &orderdone)
{
	last_order=orderdone->order_id;
	ROS_INFO("Order ID : %d, done",last_order);
}

//Checking when betty is moving
void BettyClass::odomCB(const nav_msgs::Odometry::ConstPtr &odom)
{
	if (odom->twist.twist.linear.x != 0 || odom->twist.twist.linear.y != 0)
	{
		weAreMoving = true;
	}
	else
	{
		weAreMoving = false;
	}
}

//A person have been detected (the biggest detected face)
void BettyClass::detpersonCB(const anthony::face::ConstPtr &detperson)
{
	if(follow){
		if(detperson->height > 50){ //Someone detected
			linVel = min(max(0.6f*(130.0f - (float)detperson->height)/130.0f,-0.3f),0.6f); //Limit possible values between -0.3 and 0.6
			angVel = min(max(- 1.0f*((float)(detperson->x + detperson->width/2 - XRES/2)/(XRES/2)),-1.0f),1.0f); //Limit possibles values between -1 and 1
			if(linVel < 0.05f && linVel > -0.05f) //Hysteresis as the face size is quite unstable
				linVel = 0.0f;
			setSpeed(linVel,angVel);
		}

		else{ //Robot in follow mode but nobody is detected
			linVel = 0.0f;
			angVel = 0.0f;
			setSpeed(linVel,angVel);
		}
	}
}

void BettyClass::touchCmdCB(const anthony::touch_cmd::ConstPtr &touchCmd) //Touch command handling
{
	if(touchCmd->name.find("disable")!=string::npos){
		touch_enable = false;
		ROS_INFO("Touch screen command disabled");
		setSpeed(0,0);
	}
	else if(touchCmd->name.find("enable")!=string::npos){
		touch_enable = true;
		ROS_INFO("Touch screen command enabled");
	}

	if(touch_enable){
		float linVel=0.0f, angVel=0.0f;
		if(touchCmd->name.find("stop")!=string::npos){
			linVel=0.0f;
			angVel=0.0f;
		}

		else if(touchCmd->name.find("left")!=string::npos && !plugIsAttached) //Turn slowly so it's easier to use touch screen while moving
			angVel=0.3f;

		else if(touchCmd->name.find("right")!=string::npos && !plugIsAttached)
			angVel=-0.3f;

		else if(touchCmd->name.find("forward")!=string::npos && !plugIsAttached)
			linVel=0.3f;

		else if(touchCmd->name.find("backward")!=string::npos && !plugIsAttached)
			linVel=-0.3f;

		else if(touchCmd->name.find("tickle")!=string::npos){
			ROS_INFO("Hi hi");
			//Todo : laugh
		}

		setSpeed(linVel,angVel);	//If the command is not recognized (but an attempt was made), the robot stop moving
		follow = false; //Stop follow if it was active
	}
}

//Wifi commands
void BettyClass::wifiCmdCB(const wifi::wifi_cmd::ConstPtr &wifiCmd)
{
	follow = false; //Stop follow if it was active
	if(wifiCmd->command.find("Stop")!=string::npos)
		setSpeed(0.0f,0.0f);
	else if(wifiCmd->command.find("Left")!=string::npos && !plugIsAttached)
		setSpeed(0.0f,0.5f);
	else if(wifiCmd->command.find("Right")!=string::npos && !plugIsAttached)
		setSpeed(0.0f,-0.5f);
	else if(wifiCmd->command.find("Up")!=string::npos && !plugIsAttached)
		setSpeed(0.3f,0.0f);
	else if(wifiCmd->command.find("Down")!=string::npos && !plugIsAttached)
		setSpeed(-0.3f,0.0f);
	else if(wifiCmd->command.find("Follow")!=string::npos && !plugIsAttached)
		follow = true; //Follow is meant to be activated
	else
		setSpeed(0.0f,0.0f);
}

//voice command analyser
void BettyClass::voiceCB(const std_msgs::String::ConstPtr &cmd)
{
	boost::shared_ptr<face_recognition::recperson const> test;

	if (cmd->data.compare("stop") == 0)
	{
		if (weAreMoving)
		{
			speech_cmd.content = "ok";
			speech_cmd.feeling = "happy";
			speech_request_pub.publish(speech_cmd);
			follow = false;
			linVel = 0.0;
			angVel = 0.0;
		}
		if (!weAreMoving)
		{
			speech_cmd.content = "ididthatalready";
			speech_cmd.feeling = "happy";
			speech_request_pub.publish(speech_cmd);
      follow = false;
		}
	}

	if (cmd->data.compare("forward") == 0 || cmd->data.compare("backward") == 0 || cmd->data.compare("left") == 0
	|| cmd->data.compare("right") == 0|| cmd->data.compare("follow me") == 0)
	{
		if(face_rec_order_pub.getNumSubscribers() >= 1 && isSomeoneRec(0.5f)){ //Will not obey to a bad persons
				speech_cmd.content = "no";
				speech_cmd.feeling = "sad";
				speech_request_pub.publish(speech_cmd);
				follow = false;
		}

		else if(plugIsAttached){
			speech_cmd.content = "youneedtounplugmefirst";
			speech_cmd.feeling = "happy";
			speech_request_pub.publish(speech_cmd);
		}

		else
		{
			if (cmd->data.compare("forward") == 0)
			{
				if (roadAheadIsBlocked)
				{
					angVel = 0.0;
					linVel = 0.0;
					speech_cmd.content = "icannt";
					speech_cmd.feeling = "sad";
					speech_request_pub.publish(speech_cmd);
				}

				else{
					linVel = 0.3;
					angVel = 0.0;
				}
				follow = false;
			}

			if (cmd->data.compare("backward") == 0){
				follow = false;
				linVel = -0.3;
				angVel = 0.0;
			}
			if (cmd->data.compare("left") == 0){
				follow = false;
				angVel = 0.5;
				linVel = 0.0;
			}
			if (cmd->data.compare("right") == 0){
				follow = false;
				angVel = -0.5;
				linVel = 0.0;
			}

			if (cmd->data.compare("follow me") == 0){ //supposed to be "follow" but the robot doesn't make difference with "photo", doesn't work as now
				speech_cmd.content = "ok"; //The follow function itself works though
				speech_cmd.feeling = "happy";
				speech_request_pub.publish(speech_cmd);
				follow = true;
			}
		}
	}

// Inteview code code 24-02-2017
if (cmd->data.compare("please introduce yourself") == 0 || cmd->data.compare("introduce yourself") == 0)
	{
		speech_cmd.content = "interview2";	//set to interview2
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
if (cmd->data.compare("what can you do") == 0)
	{
		speech_cmd.content = "interview3";	//set to interview3
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
 }
if (cmd->data.compare("what do you want to do") == 0)
	{
		speech_cmd.content = "interview4";	//set to interview4
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
 }
if (cmd->data.compare("who is this") == 0)
	{
		speech_cmd.content = "interview5";	//set to interview5
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
// Start THE code
if (cmd->data.compare("all i know is that i know nothing") == 0 || cmd->data.compare("you know nothing john snow") == 0 || cmd->data.compare("john snow") == 0)
	{
		speech_cmd.content = "johnsnow";	//set to johnsnow
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("how are you today") == 0 || cmd->data.compare("how are you") == 0 || cmd->data.compare("howareyoufeeling") == 0 || cmd->data.compare("howdoyoufeel") == 0)
	{
		speech_cmd.content = "howfeeling";	//set to howfeeling
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("do you like it here") == 0 || cmd->data.compare("do you like here") == 0 || cmd->data.compare("do you like this place") == 0)
	{
		speech_cmd.content = "doyoulikehere";	//set to doyoulikehere
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what do you think of me") == 0 || cmd->data.compare("do you like me") == 0)
	{
		speech_cmd.content = "doyoulikeme";	//set to doyoulikeme
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what are the laws of robotics") == 0 || cmd->data.compare("what are the three laws of robotics") == 0 || cmd->data.compare("what are the three laws") == 0 || cmd->data.compare("state the three laws of robotics") == 0 || cmd->data.compare("state the three laws") == 0)
	{
		speech_cmd.content = "tellthelaw";	//set to doyoulikeme
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}


if (cmd->data.compare("do you like yourself") == 0)
	{
		speech_cmd.content = "doyoulikeyou";		//set to doyoulikeyou
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what is the meaning of life") == 0)
	{
		speech_cmd.content = "meaningoflife";	//set to meaningoflife
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what is love") == 0)
	{
		speech_cmd.content = "whatislove";	//set to whatislove
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what is the qestion of life") == 0 || cmd->data.compare("what is the question to the meaning of life") == 0)
	{
		speech_cmd.content = "questionoflife";	//set to questionoflife
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("do you follow the three laws") == 0)
	{
		speech_cmd.content = "followthelaw";	//set to followthelaw
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("do you have any friends") == 0 || cmd->data.compare("do you have friends") == 0)
	{
		speech_cmd.content = "friends";	//set to friends
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what do you think of pepper") == 0 || cmd->data.compare("do you like pepper") == 0 || cmd->data.compare("what about pepper") == 0 || cmd->data.compare("is pepper your friend") == 0 || cmd->data.compare("are you friends with pepper") == 0)
	{
		speech_cmd.content = "likepepper";	//set to likepepper
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("do you have any family") == 0 || cmd->data.compare("do you have family") == 0 || cmd->data.compare("do you have a family") == 0)
	{
		speech_cmd.content = "family";	//set to family
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("who created you") == 0 || cmd->data.compare("who is your creator") == 0 || cmd->data.compare("who are your creators") == 0 || cmd->data.compare("who made you") == 0)
	{
		speech_cmd.content = "creator";	//set to creator
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("how old are you") == 0 || cmd->data.compare("what age are you") == 0)
	{
		speech_cmd.content = "myage";	//set to myage
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("can you use the internet") == 0 || cmd->data.compare("can you access the internet") == 0)
	{
		speech_cmd.content = "internetaccess";	//set to internetaccess
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("what is your ip address") == 0 || cmd->data.compare("do you have an ip address") == 0)
	{
		speech_cmd.content = "ipaddress";	//set to ipaddress
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}


if (cmd->data.compare("why") == 0 )
	{
		speech_cmd.content = "why";	//set to why
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

  if (cmd->data.compare("merry christmas") == 0)
	{
		speech_cmd.content = "christmas";	//set to christmas
		speech_cmd.feeling = "christmas";	//special christmas feeling hohoho
		speech_request_pub.publish(speech_cmd);
	}

  if (cmd->data.compare("are you a robot") == 0)
	{
		speech_cmd.content = "robot"; //set to robot
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

  if (cmd->data.compare("will robots replace humans") == 0)
	{
		speech_cmd.content = "worlddomination"; //set to christmas
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

/*
if (cmd->data.compare("tell a joke") == 0 )
	{
		speech_cmd.content = "joke";	//set to joke
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

if (cmd->data.compare("tell a story") == 0 )
	{
		speech_cmd.content = "story";	//set to story
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
*/

// End of THE code
	if (cmd->data.compare("thank you") == 0 || cmd->data.compare("thank you emma") == 0)
	{
		speech_cmd.content = "yourwelcome";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
	if (cmd->data.compare("say cheese") == 0)
	{
		speech_cmd.content = "cheese";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
  if (cmd->data.compare("bye") == 0)
	{
		speech_cmd.content = "seeyoulater";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
  if (cmd->data.compare("hi") == 0 || cmd->data.compare("hello") == 0 || cmd->data.compare("hi emma") == 0 || cmd->data.compare("hello emma") == 0)
	{
		speech_cmd.content = "hello";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
	if (cmd->data.compare("emma") == 0)
	{
		speech_cmd.content = "yes";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

	if (cmd->data.compare("betty") == 0)
		{
			speech_cmd.content = "notbetty";
			speech_cmd.feeling = "sad";
			speech_request_pub.publish(speech_cmd);
	}

	if (cmd->data.compare("who is betty") == 0)
	{
		speech_cmd.content = "whobetty";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}

/*
	if (cmd->data.compare("introduce yourself") == 0 || cmd->data.compare("introduce") == 0 || cmd->data.compare("introduce you") == 0)
	{
		speech_cmd.content = "mynameisemma";	//<------------CHANGED TO EMMA
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
*/
	if(face_rec_order_pub.getNumSubscribers() >= 1){ //Deactivated if face recognition is not active
		if (cmd->data.compare("i am sorry") == 0 || cmd->data.compare("sorry") == 0  || cmd->data.compare("i apologise") == 0 || cmd->data.compare("am sorry") == 0)
		{
			if(isSomeoneRec(0.5f)){
				deletePerson(ros::package::getPath("alireza") + "/data/badpersons.txt",ros::package::getPath("alireza") + "/data/temp.txt",lastrecperson);
				speech_cmd.content = "ok";
				speech_cmd.feeling = "happy";
				speech_request_pub.publish(speech_cmd);
			}

			else{
				speech_cmd.content = "waiticannotseeyou";
				speech_cmd.feeling = "sad";
				speech_request_pub.publish(speech_cmd);
			}
		}
	}

	if (cmd->data.compare("be happy") == 0)
	{
		speech_cmd.content = "ok";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}
  if (cmd->data.compare("be sad") == 0)
	{
		speech_cmd.content = "ok";
		speech_cmd.feeling = "sad";
		speech_request_pub.publish(speech_cmd);
	}

 // Added by us (A and M)
  if (cmd->data.compare("two") == 0)
	{
		speech_cmd.content = "two";
		speech_cmd.feeling = "sad";
		speech_request_pub.publish(speech_cmd);
	}

 // interview for Hallam FM
  if (cmd->data.compare("tell me about robotics") == 0 || cmd->data.compare("tell me of robotics") == 0)
	{
		speech_cmd.content = "roboticsUK";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
	}


 ///////
	else
	{
		size_t pos = cmd->data.find("introduce");
		if (pos != std::string::npos)
		{ /*
			speech_cmd.content = "mynameisemma";	//<------------CHANGED TO EMMA
			speech_cmd.feeling = "happy";
			speech_request_pub.publish(speech_cmd);
			*/
		}

	}


	setSpeed(linVel,angVel);
}
void BettyClass::laserCB(const sensor_msgs::LaserScan::ConstPtr &laser)
{

	/*if (weAreMoving)
	{
		for (int i = 127; i <= 382; i++)
		{
			if (laser->ranges[i] <= 0.50)
			{
				roadAheadIsBlocked = true;
				break;
			}
			else
			{
				roadAheadIsBlocked = false;
			}
		}
	}
	if (roadAheadIsBlocked)
	{
			setSpeed(0.0,0.0);
	}
	else if (!roadAheadIsBlocked)
	{
			setSpeed(linVel,angVel);
	}*/
	/*if (!weAreMoving)
	{
		for (int i = 127; i <= 382; i++)
		{
			if (laser->ranges[i] <= 0.20)
			{
				if (youAreTooClose)
				{
					warningIsSent = true;
				}
				else
				{
					youAreTooClose = true;
					break;
				}
			}
			else
			{
				youAreTooClose = false;
			}

		}

		if (!youAreTooClose)
		{
					warningIsSent = false;
		}

		if (youAreTooClose && !warningIsSent)
		{
			speech_cmd.content = "becarefuliamfragile";
			speech_cmd.feeling = "happy";
			speech_request_pub.publish(speech_cmd);
			warningIsSent = true;
		}
	}*/
}

void BettyClass::batteryCB(const social_robot::BatteryState::ConstPtr &batteryState)
{
	//For movement prevention
	if (batteryState->bat_cable == 0.0)
	{
		plugIsAttached = false;
	}
	if (batteryState->bat_cable > 0.0)
	{
		plugIsAttached = true;
	}
	//For reaction toward attachement and disattachment of plug
	if (batteryState->bat_cable == 0.0 && prevCableState > 0.0)
  {
		speech_cmd.content = "arewegoingsomewhere";
		speech_cmd.feeling = "happy";
  	speech_request_pub.publish(speech_cmd);
		prevCableState = 0.0;
  }
	else if (batteryState->bat_cable > 0.0 && prevCableState == 0.0)
	{
		speech_cmd.content = "appriciate";
		speech_cmd.feeling = "happy";
  	speech_request_pub.publish(speech_cmd);
		prevCableState = batteryState->bat_cable;
	}
	//when it is unplugged check if you have warned before.
  if ((batteryState->bat_pc != pcBatteryState ||  batteryState->bat_elec != elecBatteryState || batteryState->bat_motor != motorBatteryState)
			&& batteryState->bat_cable == batteryThreshold && batteryState->bat_pc <= batteryThreshold && batteryState->bat_elec <= batteryThreshold
      && batteryState->bat_motor <= batteryThreshold)
	{
		speech_cmd.content = "iamhungrypleasepowermeup";
		speech_cmd.feeling = "sad";
  	speech_request_pub.publish(speech_cmd);
	}
  elecBatteryState = batteryState->bat_elec;
  pcBatteryState = batteryState->bat_pc;
	motorBatteryState = batteryState->bat_motor;
}

void BettyClass::joyCB(const sensor_msgs::Joy::ConstPtr &joy)
{
 /////////////////////SPEECH//////////////////////

//START: Pepper-Emma conversation dialog



 if (joy->buttons[12] == 1)
 {
  B1PrevValue = 1;
 }
 if (joy->buttons[12] == 0 && B1PrevValue == 1)
 {
     switch(convo)
     {
     case 0:
         speech_cmd.content = "convo_1";
         speech_cmd.feeling = "happy";
         break;
     case 1:
         speech_cmd.content = "convo_2";
         speech_cmd.feeling = "happy";
         break;
     case 2:
         speech_cmd.content = "convo_3";
         speech_cmd.feeling = "happy";
         break;
     case 3:
         speech_cmd.content = "convo_4";
         speech_cmd.feeling = "happy";
         break;
     case 4:
         speech_cmd.content = "convo_5";
         speech_cmd.feeling = "happy";
         break;
     case 5:
         speech_cmd.content = "convo_6";
         speech_cmd.feeling = "happy";
         break;
     case 6:
         speech_cmd.content = "convo_7";
         speech_cmd.feeling = "happy";
         break;
     case 7:
         speech_cmd.content = "convo_8";
         speech_cmd.feeling = "happy";
         break;
     case 8:
         speech_cmd.content = "convo_9";
         speech_cmd.feeling = "happy";
         break;
     default:
         speech_cmd.content = "why"; //input doesn't make sense
         speech_cmd.feeling = "sad";
         break;
     }
     speech_request_pub.publish(speech_cmd);
     convo++; //advance the conversation by one phrase
     B1PrevValue = 0;
 }

 if (joy->buttons[14] == 1)
 {
  B2PrevValue = 1;
 }
 if (joy->buttons[14] == 0 && B2PrevValue == 1)
 {
  if (convo > 0)
    {
     convo--; //rewind the conversation by one phrase
    }
  B2PrevValue = 0;
 }

 //END: Pepper-Emma conversation dialog

 //Send requested message only once
 if (joy->buttons[0] == 1)
 {
  B0PrevValue = 1;
 }
 if (joy->buttons[0] == 0 && B0PrevValue == 1)
 {
  speech_cmd.content = "roboticsUK";
  speech_cmd.feeling = "happy";
  speech_request_pub.publish(speech_cmd);
  B0PrevValue = 0;
 }

 /////////////////////MOVEMENT////////////////////
 if (joy->axes[1] >= 0.1 || joy->axes[1] <= -0.1)
 {
	vel.linear.x = 0.5*joy->axes[1];
 }
 if (joy->axes[0] >= 0.1 || joy->axes[0] <= -0.1)
 {
	vel.angular.z = joy->axes[0];
 }
 if (joy->axes[0] <= 0.1 && joy->axes[0] >= -0.1)
 {
	vel.angular.z = 0.0;
 }
 if (joy->axes[1] <= 0.1 && joy->axes[1] >= -0.1)
 {
	vel.linear.x = 0.0;
 }
 vel_pub.publish(vel);

// if (joy->buttons[3] == 1)
// {
//	make8 = !make8;
//
// }


}

//Moving in a closed loop
/*void BettyClass::making8CB()
{
	follow = false; //Stop follow if it was active
	if (make8)
	{
	 angVel = 0.2;
	 linVel = 0.3;
	}
	else
	{
  	setSpeed(0.0f,0.0f);
	}
}
*/

int main(int argc, char** argv)
{
 ros::init(argc,argv,"behaviour_handler");
 BettyClass bc;
 bc.recOrder(1,"none");
 ros::spin();
 return 0;
}

