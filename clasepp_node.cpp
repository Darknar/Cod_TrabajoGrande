#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

//? Clase PurePursuit

class PurePursuit {
	private:
		ros::NodeHandle nh;
		ros::Subscriber path_sub;
		ros::Subscriber odom_sub;
		ros::Publisher cmd_vel_pub;
		nav_msgs::Path path;
		double lookahead_distance;

	public:
		//? Constructor
			PurePursuit() {
				path_sub = nh.subscribe("/move_base/NavfnROS/plan", 1, &PurePursuit::pathCallback, this);
				odom_sub = nh.subscribe("/odom", 1, &PurePursuit::PosicionRobot, this);
				cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
				lookahead_distance = 0.5;
			}

		//? Metodos
			void pathCallback(const nav_msgs::PathConstPtr &msg) {
				path = *msg;
			}

			void PosicionRobot(double &X, double &Y){
				X = odom.pose.pose.position.x;
				Y = odom.pose.pose.position.y;
			}

			double distancia(double X, double Y){
				return sqrt(pow(X, 2) + pow(Y,2));
			}


			double getClosestWaypoint() {
				double closest_distance = INT_MAX;
				int closest_waypoint = 0;
				for (int i = 0; i < path.poses.size(); i++) {

					double distance = distancia(path.poses[i].pose.position.x, path.poses[i].pose.position.y);

					if (distance < closest_distance) {
						closest_distance = distance;
						closest_waypoint = i;
					}
				}
				return closest_waypoint;
			}


			int getLookaheadWaypoint(int closest_waypoint) {
				int lookahead_waypoint = closest_waypoint;
				for (int i = closest_waypoint; i < path.poses.size(); i++) {

					double distance = distancia(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
					ROS_INFO_STREAM("Distance " << distance);

					if (distance > lookahead_distance) {
						ROS_INFO_STREAM("Distance>LkAD. Index: " << i);
						lookahead_waypoint = i;
						break;
					}
				}
				return lookahead_waypoint;
			}


			double getAngle(int closest_waypoint, int lookahead_waypoint) {
				if (path.poses.empty()) {
					ROS_WARN("No path data found");
					return 0.0;
				}
				double dx = path.poses[lookahead_waypoint].pose.position.x - path.poses[closest_waypoint].pose.position.x;
				double dy = path.poses[lookahead_waypoint].pose.position.y - path.poses[closest_waypoint].pose.position.y;
				ROS_INFO_STREAM("dx " << dx);
				ROS_INFO_STREAM("dy " << dy);
				// dx=dy=1.0;
				return atan2(dy, dx);
			}


			void move(double angle){
				double angular_velocity = angle;
				double linear_velocity = 0.1;
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = linear_velocity;
				cmd_vel.angular.z = angular_velocity;
				cmd_vel_pub.publish(cmd_vel);
			}

			void controlLoop() {
				while (ros::ok()) {
					int closest_waypoint = getClosestWaypoint();
					ROS_INFO_STREAM("ClosestWaypoint " << closest_waypoint);

					int lookahead_waypoint = getLookaheadWaypoint(closest_waypoint);
					ROS_INFO_STREAM("LookaheadWaypoint " << lookahead_waypoint);

					double angle = getAngle(closest_waypoint, lookahead_waypoint);
					ROS_INFO_STREAM("Angle " << angle);

					if (path.poses.empty())
						ROS_WARN("No path data found");
					else
						move(angle);
					
					ros::spinOnce();
				}
			}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "pure_pursuit");
	PurePursuit pp;
	pp.controlLoop();
	return 0;
}
