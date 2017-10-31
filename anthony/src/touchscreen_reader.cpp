//Touch screen reader, read directly through /dev/input/by-id/usb-TPK_USA_LLC_Touch_Fusion_4.-event-if00
//Topics suscribed : none
//Topics published : touch_in (X Y coordinates, Touch state, Event type)
//Note : touch state and touching coordinates are sent in the same message but separatively
//It will be up to the reading node to use those accordingly

#include <ros/ros.h>
#include <anthony/touch_in.h>
#include <fcntl.h>
#include <linux/input.h>
#include <errno.h>
#define EVENT_DEVICE "/dev/input/by-id/usb-TPK_USA_LLC_Touch_Fusion_4.-event-if00"//Change if touch screen is another device (+check permissions)
#define TOUCH_MAX_VAL 4094  //tested on touch screen
#define XRES 1280
#define YRES 720 //Adapt to the target touch screen

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "touchscreen_reader");
	ros::NodeHandle nh;
	ros::Publisher touch_pub = nh.advertise<anthony::touch_in>("touch_in",50);
	anthony::touch_in tsData;

	struct input_event ev;
	int fd;
	char name[256] = "Unknown";

	//Opening the device
	fd = open(EVENT_DEVICE, O_RDONLY);
	if (fd == -1) {
		ROS_ERROR("Touchscreen is not available. Make sure the alias open_touchscreen have been used at least once this session");
		ROS_ERROR("Error with %s, type : %s\n", EVENT_DEVICE,strerror(errno));
		return EXIT_FAILURE;
	}

	//Infos about Device
  ioctl(fd, EVIOCGNAME(sizeof(name)), name);
  ROS_INFO("Reading from:");
  ROS_INFO("device file = %s", EVENT_DEVICE);
  ROS_INFO("device name = %s", name);

	while(ros::ok())
	{
		const size_t ev_size = sizeof(struct input_event);
		read(fd, &ev, ev_size);

		if(ev.type == EV_ABS && ev.code == ABS_X){ //Retrieves X data
			tsData.x = ev.value*XRES/TOUCH_MAX_VAL; //convert to screen size
			read(fd, &ev, ev_size);
			if(ev.type == EV_ABS && ev.code == ABS_Y){//Retrieves Y data
				tsData.y = (TOUCH_MAX_VAL-ev.value)*YRES/TOUCH_MAX_VAL; //convert to screen size
				tsData.ev_type = "COOR";
				touch_pub.publish(tsData); //With this conversion the reference (0,0) is top left corner
			}
		}

		else if(ev.type == EV_KEY && ev.code == BTN_TOUCH){ //Retrieves Touch new value
			tsData.touch = ev.value;
			tsData.ev_type = "TOUCH";
			touch_pub.publish(tsData);
		}
	}

	close(fd);
}
