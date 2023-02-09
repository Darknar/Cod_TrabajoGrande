#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <vector>
#include <math.h>

class Punto{
	private:
		double x;
		double y;
		bool visitado;

	public:
		//? Constructor
			Punto(double x, double y) : x(x), y(y), visitado(false) {}

		//? Metodos
			double distancia(){
				return sqrt(pow(X, 2) + pow(Y,2));
			}

			//? Getters
				double getX(){ return x; }
				double getY(){ return y; }
				bool getVisitado(){ return visitado; }

			//? Setters
				bool setVisitado(bool visitado){
					this->visitado = visitado;
				}

};

class PurePursuit {
	private:
		const double Velocidad_lineal;

		ros::NodeHandle nh;
		ros::Subscriber path_sub;
		ros::Subscriber odom_sub;
		ros::Subscriber baselink_sub;
		ros::Publisher cmd_vel_pub;
		nav_msgs::Path path;

		double Distancia_Futura;

		std::vector<Punto> puntos;

	public:
		//? Constructor
			PurePursuit() : Velocidad_lineal(0.1) {
				path_sub = nh.subscribe("/move_base/NavfnROS/plan", 1, &PurePursuit::pathCallback, this);
				baselink_sub = nh.subscribe("/base_link", 1, &PurePursuit::callbase_link, this);
				odom_sub = nh.subscribe("/odom", 1, &PurePursuit::callback_odom, this);
				cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
				Distancia_Futura = 0.5;
			}

		//? Metodos
			void pathCallback(const nav_msgs::PathConstPtr &msg) {
				path = *msg;
				puntos = std::vector<Punto>();
				for (int i = 0; i < path.poses.size(); i++) {
					puntos.push_back(Punto(path.poses[i].pose.position.x, path.poses[i].pose.position.y));
				}
			}

			void callback_odom(const nav_msgs::Odometry::ConstPtr& msg){
				odomPose2D.x = msg->pose.pose.position.x;
				odomPose2D.y = msg->pose.pose.position.y;
			}

			void callbase_link(const geometry_msgs::PoseStamped::ConstPtr& msg){
				base_linkPose2D.x = msg->pose.position.x;
				base_linkPose2D.y = msg->pose.position.y;
			}

			double puntoCercano() {
				double distancia_mas_cercana = INT_MAX;
				int Punto_con_distancia_mas_cercana = 0;
				for (int i = 0; i < puntos.size(); i++) {

					double distance = puntos[i].distancia();

					if (distance < distancia_mas_cercana) {
						distancia_mas_cercana = distance;
						Punto_con_distancia_mas_cercana = i;
					}
				}
				return Punto_con_distancia_mas_cercana;
			}


			int getLookaheadWaypoint(int Punto_con_distancia_mas_cercana) {
				int Punto_Futuro = Punto_con_distancia_mas_cercana;
				for (int i = Punto_con_distancia_mas_cercana; i < puntos.size(); i++) {

					double distance = puntos[i].distancia();
					ROS_INFO_STREAM("Distance " << distance);

					if (distance > Distancia_Futura) {
						ROS_INFO_STREAM("Distance>LkAD. Index: " << i);
						Punto_Futuro = i;
						break;
					}
				}
				return Punto_Futuro;
			}


			double getAngle(int Punto_con_distancia_mas_cercana, int Punto_Futuro) {
				if (puntos.empty()) {
					ROS_WARN("No path data found");
					return 0.0;
				}
				double dx = puntos[Punto_Futuro].getX() - puntos[Punto_con_distancia_mas_cercana].getX();
				double dy = puntos[Punto_Futuro].getY() - puntos[Punto_con_distancia_mas_cercana].getY();

				ROS_INFO_STREAM("dx " << dx);
				ROS_INFO_STREAM("dy " << dy);

				return atan2(dy, dx);
			}


			void move(double Angulo, int Punto_con_distancia_mas_cercana){
				double Velocidad_Angular = (2*puntos[Punto_con_distancia_mas_cercana].getY()) / pow(Distancia_Futura, 2) * Velocidad_lineal; 
				// Velocidad angular = 2y / r^2 * v

				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = Velocidad_lineal;
				cmd_vel.angular.z = Velocidad_Angular;
				cmd_vel_pub.publish(cmd_vel);
			}

			void controlLoop() {

				while (ros::ok()) {
					int Punto_con_distancia_mas_cercana = puntoCercano();
					ROS_INFO_STREAM("ClosestWaypoint " << Punto_con_distancia_mas_cercana);

					int Punto_Futuro = getLookaheadWaypoint(Punto_con_distancia_mas_cercana);
					ROS_INFO_STREAM("LookaheadWaypoint " << Punto_Futuro);

					double Angulo = getAngle(Punto_con_distancia_mas_cercana, Punto_Futuro);
					ROS_INFO_STREAM("Angle " << Angulo);

					if (Puntos.empty())
						ROS_WARN("No path data found");
					else
						move(Angulo, Punto_con_distancia_mas_cercana);
					
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
