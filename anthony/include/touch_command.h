#ifndef TOUCH_COMMAND_H
#define TOUCH_COMMAND_H

#include <ros/ros.h>
#include <anthony/touch_in.h>
#include <anthony/touch_cmd.h>
#include <QtGui>
#include <QPainter>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class TouchCommand : public QWidget {
	typedef struct {
		std::vector<QPoint> track;
		float size_ratio;
		std::string name;
	}
	Model;
	
  Q_OBJECT
public:
	TouchCommand(int argc=0, char **argv=NULL, QWidget *parent = 0);
	~TouchCommand();
protected:
	ros::NodeHandle nh;
	std::vector<QPoint> sample; //The sample from touch screen
	std::vector<QPoint> spl_disp; //Sample copy for display 
	std::vector<Model> index; //Models created from bmp files at initialisation
	bool touch_state; //Touch state, is the user touching the screen ?
	int rec_id;
	ros::Subscriber touch_in_sub;
	ros::Publisher touch_cmd_pub;
	void paintEvent(QPaintEvent * event); //Update display
	std::vector<QPoint> trackToDisplay(std::vector<QPoint> track); //Return a display version of a normalized track (size, center)
	void drawTrack(QPainter *p, std::vector<QPoint> track, bool draw_line, QColor color); //Draw a track
	void touch_inCB(const anthony::touch_in::ConstPtr& tin); //Received Touch screen message
	//ModelReader
	float normalizeTrack(std::vector<QPoint> &track); //Normalize geometry data of the track (size, center)
	Model createModel(const cv::Mat img, const std::string imgname); //Create a model from a bmp file
	void loadInIndex(const std::string imgname, const std::string dirname); //Load model in the index
	float getModelSampleDist(const int index_nb); //Get matching by distance between model and sample
	std::string findClosest(); //Find closest model to sample
};

#endif
