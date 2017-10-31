//Touch command recognition, read touch in topic to record tracks and associate those to model commands
//Topics suscribed : touch_in (X Y coordinates, Touch state, Event type)
//Topics published : touch_cmd (command name, command id)

#include "../include/touch_command.h"
#include <iostream>
#include <dirent.h>
#include <cmath>
#include <climits>
#include <cfloat>
#include <ros/package.h>

using namespace cv;
using namespace std;

#define SAMPLE_MIN_SIZE 20 //Minimum allowed sample vector size, else discarded
#define MATCH_TRIGGER 15//Under that value, sample does not match any model (qualitative, from tests, depends of DIST_FACTOR)
#define DIST_FACTOR 1 //When Dist factor is settled lower, the recognition tend to favors simple symbols. When higher, it favors complex ones
//With the current set it seems to perform best between 0.75 and 1
#define XRES 720
#define YRES 1280 //Adapt to touch screen resolution

TouchCommand::TouchCommand(int argc, char **argv, QWidget *parent) : QWidget(parent) {
	DIR *dir = NULL;
	struct dirent *ent;
	string dirname;
	
	if(argc == 2)
		dirname = string(argv[1]);
	else
		dirname = ros::package::getPath("anthony") + "/Models/";

	if ((dir = opendir(dirname.c_str())) != NULL) {
		index.clear(); //make sure the index is empty
		while ((ent = readdir (dir)) != NULL){
			const string d_str(ent->d_name);
			if(d_str.find(".bmp")!=string::npos)//is a model
				loadInIndex(d_str,dirname);
		}
		closedir (dir);
	}

	else {
		ROS_ERROR("Couldn't open Models folder, closing node");
		ros::shutdown();
	}
 
	this->resize(320,240);
	this->setWindowTitle(QApplication::translate("toplevel","Touch Command"));	
	this->show();
	sample.clear();
	spl_disp.clear();
	touch_in_sub = nh.subscribe("touch_in", 1000, &TouchCommand::touch_inCB,this);
	touch_cmd_pub = nh.advertise<anthony::touch_cmd>("touch_cmd",5);
	rec_id = -2;
}

TouchCommand::~TouchCommand() {}

void TouchCommand::paintEvent(QPaintEvent * event) {
  QPainter p(this);
  p.setBrush(QColor(Qt::white));
  p.drawRect(event->rect().adjusted(5,5,-5,-5));
	
	if(rec_id >= 0 && rec_id < index.size()) //command recognized
		drawTrack(&p,trackToDisplay(index[rec_id].track),false,QColor(Qt::green));

	else if(rec_id == -1)//command not recognized or sample too short
		drawTrack(&p,spl_disp,true,QColor(Qt::red));

	else //user currently drawing
		drawTrack(&p,spl_disp,true,QColor(Qt::blue));
}

std::vector<QPoint> TouchCommand::trackToDisplay(std::vector<QPoint> track) {
	std::vector<QPoint> disp;
	disp.clear();	
	for(unsigned int i=1; i < track.size(); i++)
		disp.push_back(3.5*track[i]+QPoint(XRES/2.0f,YRES/2.0f));
	
	return disp;
}

void TouchCommand::drawTrack(QPainter* p, vector<QPoint> track, bool draw_line, QColor color) {
	QPen pen;
	pen.setWidth(7);
	pen.setColor(color);
	p->setPen(pen);
	for(unsigned int i=1; i < track.size(); i++)
		if(draw_line)
			p->drawLine(mapFromGlobal(track[i-1]),mapFromGlobal(track[i]));
		else
			p->drawPoint(mapFromGlobal(track[i]));
}

void TouchCommand::touch_inCB(const anthony::touch_in::ConstPtr& tin)
{
	if(tin->ev_type.find("COOR") != string::npos){ //Coordinates data type
		if(touch_state == false){ //New entry
			rec_id = -2;
			sample.clear();
			spl_disp.clear();
		}
		touch_state = true;
		sample.push_back(QPoint(tin->y,tin->x)); //Add new point to sample (data inverted)
		spl_disp.push_back(QPoint(tin->y,tin->x));
	}

	else if((tin->ev_type.find("TOUCH") != string::npos) && tin->touch == false){ //The user stopped input
		touch_state = false;
		ROS_INFO("The touch command %s have been recognized",findClosest().c_str());
	}
}

float TouchCommand::normalizeTrack(vector<QPoint> &track)
{
	//Get geometry datas
	float xavg=0.0f, yavg=0.0f;
	int xsize=0, ysize=0,xmin=INT_MAX,xmax=0,ymin=INT_MAX,ymax=0;

	for(unsigned int i = 0; i < track.size(); i++) {
		xavg = xavg + (float)track[i].x()/track.size(); //Average x, y to realign centre (barycentre method)
		yavg = yavg + (float)track[i].y()/track.size();
		xmin = min(xmin,track[i].x());
		xmax = max(xmax,track[i].x());
		ymin = min(ymin,track[i].y());
		ymax = max(ymax,track[i].y());
	}

	xsize = xmax - xmin; // Model size to normalize
	ysize = ymax - ymin;
	float size_ratio = (float)(ysize+1)/(xsize+1);

	//Normalize datas
	//And remove unecessary duplicate element
	for(unsigned int i = 0; i < track.size(); i++) {
		track[i].setX((track[i].x() - xavg)*100/max(xsize,ysize));
		track[i].setY((track[i].y() - yavg)*100/max(xsize,ysize));
		if(i) //i is not zero
			if(std::find(track.begin(),track.begin() + i,track[i])!=(track.begin()+i)) //Same point is uncessary
				track.erase(track.begin()+i--); //This duplicate removing method might be slow but done at index creation, which happen once
	}

	return size_ratio;
}

TouchCommand::Model TouchCommand::createModel(const Mat img, const string imgname){
	Model mod;
	mod.track.clear();

	for(int i=0; i < img.size().width; i++)
		for(int j=0; j < img.size().height; j++)
			if((int)img.at<uchar>(j,i,0)<128) //Data. For some reason OpenCV puts Y data in first dim
				mod.track.push_back(QPoint(i,j));

	mod.size_ratio = normalizeTrack(mod.track); //Normalize the model (size, center)

	string buf = imgname; //The command name is created using image name
	buf.erase(buf.end()-4,buf.end()); //remove .bmp extension, no longer useful in index
	mod.name = buf;
	return mod;
}

void TouchCommand::loadInIndex(const string imgname, const string dirname){
	Mat img = imread(dirname + imgname,CV_LOAD_IMAGE_GRAYSCALE);
	if(!img.data)
		ROS_ERROR("Error opening image\n");

	else
		index.push_back(createModel(img,imgname));

	img.release();
}

float TouchCommand::getModelSampleDist(const int idx_nb){
	float total_dist = 0.0f;
	for (unsigned int j=0; j<sample.size(); j++){ //Each point in the sample
		float dist = FLT_MAX;
		for(unsigned int i=0; i<index[idx_nb].track.size(); i++) //Retrieves closest dist between sample point and the whole model
			dist = min(dist,(float)(QLineF(sample[j],index[idx_nb].track[i])).length());
		
		total_dist = total_dist + dist/pow(sample.size(),DIST_FACTOR);
		//Distance is made less dependant of sample vector size and track vector size (not a true length but good qualitative criteria)
	}

	for (unsigned int i=0; i<index[idx_nb].track.size(); i++){ //Each point in the model
		float dist = FLT_MAX;
		for(unsigned int j=0; j<sample.size(); j++){ //Retrieves closest dist between model point and whole
			dist = min(dist,(float)(QLineF(sample[j],index[idx_nb].track[i])).length());
		}

		total_dist = total_dist + dist/pow(index[idx_nb].track.size(),DIST_FACTOR);
	}
	
	return total_dist;
}

string TouchCommand::findClosest(){
	anthony::touch_cmd tc;
	
	if(sample.size()>=SAMPLE_MIN_SIZE){
		float min_dist=FLT_MAX;
		string model_name;
		float sampr = normalizeTrack(sample); //Normalizes sample, gives ratio

		for(unsigned int i = 0; i < index.size(); i++){
			float idxr = index[i].size_ratio;
			if(((sampr/idxr) < 2 && (sampr/idxr) > 0.5) || idxr > 10 ||idxr < 0.1){//First quick sorting : ratio must be close enough or model must be narrow
				float gdist = getModelSampleDist(i);
				if(min_dist > gdist){
					min_dist = gdist;
					model_name = index[i].name;
					rec_id = i;
				}
				ROS_INFO("Distance between %s model and sample : %f",index[i].name.c_str(),gdist);
			}
		}

		if(min_dist <= MATCH_TRIGGER){ //"Decent accuracy" sample
			tc.name = model_name;
			tc.id = rec_id;
			touch_cmd_pub.publish(tc);	
			return model_name;
		}

		else{
			rec_id = -1;
			tc.name = "nomatch";
			tc.id = rec_id;
			touch_cmd_pub.publish(tc);
			return "nomatch";
		}
	}

	else{
		rec_id = -1;
		tc.name = "nomatch";
		tc.id = rec_id;
		touch_cmd_pub.publish(tc);
		return "shortsample";
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "touch_command");
	QApplication app(argc, argv);
  TouchCommand tcom(argc,argv);
	while(ros::ok()) {
		app.processEvents();
		app.sendPostedEvents();
		tcom.repaint();
		ros::spinOnce();
		if(!tcom.isVisible()){
			ROS_INFO("Widget have been closed, shutting down node.");
			ros::shutdown();
		}
	}
}
