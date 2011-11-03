#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

double T = 0.3; //sec
double dt = 0.1; //ms

double x_ = 0;
double y_ = 0;
double z_ = 0;
double epsilon = 0.0001;

ros::Publisher twistPublisher;

double lowPassFilter(double x, double y0, double dt, double T)          // Taken from http://en.wikipedia.org/wiki/Low-pass_filter
{
   double res = y0 + (x - y0) * (dt/(dt+T));
   
   if ((res*res) <= epsilon)
	res = 0;
   return res;
}

void twistCallback(const geometry_msgs::Twist& twist){
    double x =  twist.linear.x;
    double y = twist.linear.y;
	double z = twist.angular.z;

	x_ = lowPassFilter(x, x_, dt, T);
	y_ = lowPassFilter(y, y_, dt, T);
	z_ = lowPassFilter(z, z_, dt, T);


	geometry_msgs::Twist t;
	t.linear.x = x_;
	t.linear.y = y_;
	t.angular.z = z_;

	twistPublisher.publish(t);

}

int main(int argc, char **argv)
{

    /// Receives Twist messages for the base.
	ros::Subscriber baseCommandSubscriber;
	/// Publishes Odometry messages
	ros::Publisher baseOdometryPublisher;

	ros::init(argc, argv, "lowpass_filter");
	ros::NodeHandle n;
    /* setup input/output communication */

        n.param("T", T, 0.3);
	n.param("dt", dt, 0.1);

	ros::Subscriber twistSubscriber = n.subscribe("move_base/cmd_vel", 1000, &twistCallback);
	twistPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

	/* coordination */
	ros::Rate rate(100); //Input and output at the same time... (in Hz)
	while (n.ok()){
		ros::spinOnce();

		rate.sleep();
	}

  return 0;
}

