#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "OpenniLaserScanner.h"

class MyMwListener : public OpenniLaser::OpenniLaserScanner::Listener
{
public:
	sensor_msgs::LaserScan scan;
	void readyForNextData(OpenniLaser::OpenniLaserScanner* scanner)
	{
		openni::VideoFrameRef frame;
		scan = scanner->getNextData(frame);
	}
};

int main(int argc, char **argv)
{
	 ros::init(argc, argv, "openni2_laserscan");

	  ros::NodeHandle n;
	  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("openni2_laserscan", 50);

	  int count = 0;
	  ros::Rate r(30.0);

		OpenniLaser::OpenniLaserScanner laserScanner;

		if (!laserScanner.isValid())
		{
			printf("openni_laserscan: error in initialization\n");
			return 1;
		}
		printf("openni_laserscan: running\n");

		MyMwListener myListener;

		laserScanner.setListener(myListener);

	  while(n.ok()){
	    scan_pub.publish(myListener.scan);
	    ++count;
	    r.sleep();
	  }

	  laserScanner.resetListener();
	  printf("openni_laserscan: stopped\n");
	  return 0;
}
