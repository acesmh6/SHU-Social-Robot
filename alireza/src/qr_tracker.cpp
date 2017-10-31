/*Designed By Alireza Janani*/
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include "alireza/Hri.h"
#include <std_msgs/String.h>
#include <zbar.h> 
#include <iostream> 
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <sstream>


using namespace cv; 
using namespace std; 
using namespace zbar; 

struct point
{
	float x;
	float y;
};
alireza::Hri speech_cmd;
geometry_msgs::Twist velcmd; 
void closeFunction(int sig);
std_msgs::String msg;
Mat img,hsv,imgth;
point qrPoint[4],maxPoint, minPoint, CenterPoint, FrameCenter;
string fileName;
int fileCounter = 0;
float diagonal;
bool followIsActivated = false, warningIsSent = false, takePhoto = false, isHumanReadyForPicture = false;
class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher localisation_pub;
		ros::Publisher vel_pub_;
		ros::Subscriber	voice_sub;
	  ros::Publisher speech_request_pub;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
private:
    point findMin(point po[4]);
		point findMax(point po[4]);
		void voiceCB(const std_msgs::String::ConstPtr &cmd);
public:
    ImageConverter(): it_(nh_)
    {
        //Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);
        //localisation_pub = nh_.advertise<image_analyser::ImageAnalysed>("/qr_code/position",1);
				vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",2);
				voice_sub = nh_.subscribe<std_msgs::String>("/recognizer/output",5,&ImageConverter::voiceCB,this);
        speech_request_pub = nh_.advertise<alireza::Hri>("feedback_request",1);
    }
    ~ImageConverter()
    {
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {

        cv_bridge::CvImagePtr cv_ptr;
        /*1. convert the ros_image to opencv_image*/
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        img = cv_ptr->image;
//////////////////////////////////TAKING PHOTO/////////////////////////////
					if (isHumanReadyForPicture)
					{		
						ostringstream convert;
						convert << fileCounter;
						fileName  = "image_" + convert.str() + ".jpg";
						imwrite(fileName,img);

						Mat takenImage = imread(fileName,1);

						imshow(fileName, takenImage);

						ROS_INFO("Image is taken");
						
						fileCounter++;
						isHumanReadyForPicture = false;
					}
    //////////////////////////////QR Extracting/////////////////////////////////
        //initializing the MAT will cause the negative values of Point2f goes away
         ImageScanner scanner;  
         scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); 
         namedWindow("QR_Tracker",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"   
         Mat grey = Mat::zeros(480, 640, CV_8UC1); 
         cvtColor(img,grey,CV_BGR2GRAY); 
         int width = img.cols;  
         int height = img.rows;  
         uchar *raw = (uchar *)grey.data;  
         // wrap image data  
         Image image(width, height, "Y800", raw, width * height);  
         // scan the image for barcodes  
         int n = scanner.scan(image); 
					//If nothing is detected
         if (n == 0)
         {
						//velcmd.linear.x = 0.0;
						//velcmd.angular.z = 0.0;
						//vel_pub_.publish(velcmd);
						/*if (followIsActivated && !warningIsSent)
						{
							speech_cmd.content = "waiticannotseeyou";
							speech_cmd.feeling = "sad";
							speech_request_pub.publish(speech_cmd);
							warningIsSent = true;
						}*/
						
         }
         // extract results  
         for(Image::SymbolIterator symbol = image.symbol_begin();  
         symbol != image.symbol_end();  
         ++symbol) {  
         vector<Point> vp;  
         // do something useful with results  
         cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;
         //msg.data = symbol->get_data();
         //qr_pub_.publish(msg);
           int n = symbol->get_location_size();  
         
           for(int i=0;i<n;i++){  
         vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i))); 
           }  
           RotatedRect r = minAreaRect(vp);  
           Point2f pts[4]; 
        
           r.points(pts);
					for(int i=0;i<4;i++){   
					 line(img,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);   
					 ROS_INFO("X = %f, Y = %f",pts[i].x,pts[i].y); 
	       }
					
					///////////////////////////////////FORCE STOP/////////////////////////////
					if (symbol->get_data().compare("FOLLOW") != 0)
					{
							qrPoint[0].x = 0.0;
		      		qrPoint[0].y = 0.0;
						  qrPoint[1].x = 0.0;
						  qrPoint[1].y = 0.0;
						  qrPoint[2].x = 0.0;
						  qrPoint[2].y = 0.0;
						  qrPoint[3].x = 0.0;
						  qrPoint[3].y = 0.0;
						velcmd.linear.x = 0.0;
						velcmd.angular.z = 0.0;
						vel_pub_.publish(velcmd);
					}
					/////////////////////////////FOLLOW///////////////////////////////////////
					if (symbol->get_data().compare("FOLLOW") == 0)
					 {
						if (followIsActivated)
						 {
							qrPoint[0].x = pts[0].x;
		      		qrPoint[0].y = pts[0].y;
						  qrPoint[1].x = pts[1].x;
						  qrPoint[1].y = pts[1].y;
						  qrPoint[2].x = pts[2].x;
						  qrPoint[2].y = pts[2].y;
						  qrPoint[3].x = pts[3].x;
						  qrPoint[3].y = pts[3].y;
							minPoint = findMin(qrPoint);
							 maxPoint	= findMax(qrPoint);
							 CenterPoint.x = (maxPoint.x + minPoint.x)/2;
							 CenterPoint.y = (maxPoint.y + minPoint.y)/2;
						   //find diagonal, the smaller the diagonal, the farther the distance toward the qrcode and hence the speed should be faster.
						   //In other words, magnitude of diagonal is inversely proportional to the distance to the qrcode
						   diagonal = sqrt((maxPoint.x-minPoint.x)*(maxPoint.x-minPoint.x) + (maxPoint.y - minPoint.y)*(maxPoint.y - minPoint.y));
							 velcmd.linear.x = (((diagonal)*-0.002) + 0.35)*2;		
								/*the difference between center of the frame and center of the qr-code is between -250 to 250*/
								if ((FrameCenter.x - CenterPoint.x) >= 50.0 || (FrameCenter.x - CenterPoint.x) <= -50.0)
								{
									velcmd.angular.z = (FrameCenter.x - CenterPoint.x)*0.002;
									//ROS_INFO("difference in centroid = %f",(FrameCenter.x - CenterPoint.x));
								} 
							vel_pub_.publish(velcmd);
							warningIsSent = false;
						}
					 }

					 
         }  
				 
         imshow("QR_Tracker", img); //show the frame in "MyVideo" window 
         if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop 
         { 
           cout << "esc key is pressed by user" << endl; 
         } 
    }
};

void ImageConverter::voiceCB(const std_msgs::String::ConstPtr &cmd)
{
 if (cmd->data.compare("follow me") == 0)
 {
	followIsActivated = true;
	speech_cmd.content = "okleadtheway";
	speech_cmd.feeling = "happy";
	speech_request_pub.publish(speech_cmd);
 }
 if (cmd->data.compare("stop") == 0)
 {
	followIsActivated = false;	
 }
 if (cmd->data.compare("take photo") == 0 || cmd->data.compare("take picture") == 0)
 {
	takePhoto = true;
	speech_cmd.content = "areyouready";
	speech_cmd.feeling = "happy";
	speech_request_pub.publish(speech_cmd);
 }
 if (cmd->data.compare("yes") == 0)
 {
	if (takePhoto)
	{
		isHumanReadyForPicture = true;
		speech_cmd.content = "click";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
		takePhoto = false;
	}
 }
 if (cmd->data.compare("no") == 0)
 {
	if (takePhoto)
	{
		isHumanReadyForPicture = false;
		speech_cmd.content = "ok";
		speech_cmd.feeling = "happy";
		speech_request_pub.publish(speech_cmd);
		takePhoto = false;
	}
 }
}
point ImageConverter::findMin(point po[4])
{
 point temp;
 temp.x = 480.0;
 temp.y = 640.0;
 for (int i=0; i<4; i++)
 {
	if (po[i].x <= temp.x )//&& po[i].y <= temp.y)
	{
		temp = po[i];	 
	}
 }
 return temp;
}
point ImageConverter::findMax(point po[4])
{
 point temp;
 temp.x = 0.0;
 temp.y = 0.0;
 for (int i=0; i<4; i++)
 {
	if (po[i].x >= temp.x )//&& po[i].y >= temp.y)
	{
		temp = po[i];	 
	}
 }
 return temp;
}
 //g++ main.cpp /usr/local/include/ /usr/local/lib/ -lopencv_highgui.2.4.8 -lopencv_core.2.4.8 
int main(int argc, char** argv) 
{ 
    ros::init(argc,argv,"qr_tracker");
		FrameCenter.x = 320.0; // 640/2
 		FrameCenter.y = 240.0; // 480/2
    ImageConverter ic;
    ros::spin();   
    return 0; 
}
void closeFunction(int sig){ // can be called asynchronously
  //cap.release();
  exit(EXIT_SUCCESS);
} 
