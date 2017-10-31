//This face detector reuse the OpenCV Face Cascade detector
//Topics suscribed : /usb_cam/image_raw (Camera)
//Topics published : singleface (Rectangle properties of the biggest detected face)
//									 allfaces (Rectangle properties of all faces)

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <anthony/face.h>
#include <anthony/faceArray.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <stdio.h>

#define DISPLAY_CAM //Uncomment to display detection on camera feedback window

using namespace std;
using namespace cv;

class FD_Class
{
	public:
		FD_Class():it(nh)
		{
			#ifdef DISPLAY_CAM
				namedWindow("view");
  			startWindowThread();  
			#endif
			cam_sub = it.subscribe("/usb_cam/image_raw", 1, &FD_Class::camCB, this); //camera
			face_pub = nh.advertise<anthony::face>("singleface",5); //Topic with biggest detected face
			allfaces_pub = nh.advertise<anthony::faceArray>("allfaces",5); //Topic with all detected faces
		}

		~FD_Class()
		{
			#ifdef DISPLAY_CAM
			destroyWindow("view");
			#endif
		}
	
	protected:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber cam_sub;
		ros::Publisher face_pub;
		ros::Publisher allfaces_pub;		

		anthony::face initFace(const int x, const int y, const int w, const int h);
		void camCB(const sensor_msgs::ImageConstPtr& msg);
};

anthony::face FD_Class::initFace(const int x, const int y, const int w, const int h) //Create new message containing face datas in parameters
{
	anthony::face new_face; 
	new_face.x = x;
	new_face.y = y;
	new_face.width = w;
	new_face.height = h;
	return new_face;
}

void FD_Class::camCB(const sensor_msgs::ImageConstPtr& msg) //Callback on receiving a camera frame, this is where detection is performed
{
	cv_bridge::CvImagePtr im_ptr,greys_ptr; 
	std::vector<Rect> faces;
	CascadeClassifier face_cascade;
	anthony::face single_face;
	anthony::faceArray all_faces;

	single_face.width=0;
	single_face.height=0;

  try{
		#ifdef DISPLAY_CAM
			im_ptr = cv_bridge::toCvCopy(msg,"bgr8"); //display image (if visual feedback is activated)
		#endif
		greys_ptr = cv_bridge::toCvCopy(msg,"mono8"); //compute image for detection
	}
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

	if(!face_cascade.load(ros::package::getPath("anthony") + "/Classifier/haarcascade_frontalface_alt.xml")) //Load the OpenCV cascade classifier
		ROS_WARN("Could not load the Cascade Classifier");

	face_cascade.detectMultiScale(greys_ptr->image, faces, 1.1, 3, CV_HAAR_SCALE_IMAGE, Size(30,30)); //Perform detection on the received frame

	for(int i = 0; i < faces.size(); i++) //For each found face
	{
			
			anthony::face temp_face = initFace(faces[i].x,faces[i].y,faces[i].width,faces[i].height);
			if((temp_face.width + temp_face.height) > (single_face.width + single_face.height)) //Find the largest detected face
				single_face = temp_face; //Update biggest current face if needed

			all_faces.faces.push_back(temp_face); //Add found face to array (for second message)
		  #ifdef DISPLAY_CAM
				Point pt1(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
		  	Point pt2(faces[i].x, faces[i].y);
		 		rectangle(im_ptr->image, pt1, pt2, cvScalar(0, 255, 0, 0), 1, 8, 0);
			#endif
			//ROS_INFO("Face detected, center : (%d,%d), size :(%d,%d)",faces[i].x+faces[i].width/2,faces[i].y+faces[i].height/2,faces[i].width,faces[i].height);
	}
	
	face_pub.publish(single_face);
	allfaces_pub.publish(all_faces);
 	#ifdef DISPLAY_CAM
		imshow("view", im_ptr->image);
	#endif
	waitKey(30);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_detector");
  FD_Class fdc;
  ros::spin();
}
