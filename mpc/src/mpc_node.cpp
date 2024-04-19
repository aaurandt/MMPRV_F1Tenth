#include "rclcpp/rclcpp.hpp"
#include <string>
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <OsqpEigen/OsqpEigen.h>
#include "vehicle_model.hpp"
#include "constraints.hpp"
#include "mpc.hpp"
#include "state.hpp"
#include "input.hpp"
#include "trajectory_planner.hpp"
#include <random>
#include <chrono>

#define NUM_TRAJECTORIES 216  //K
#define TRAJECTORY_LENGTH 20 //N

class MPCNode : public rclcpp::Node{
public:
   MPCNode() : Node("mpc_node") {
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      std::random_device rd;
      gen = std::mt19937(rd());
      velocity_dist = std::uniform_real_distribution<double>(0.5, 3.0);
      steering_angle_dist = std::uniform_real_distribution<double>(-0.436332, 0.436332);
      mpc_controller.setHorizon(TRAJECTORY_LENGTH);
      velocity = 0.0;
      steering_angle = 0.0;
      received_opp_odom = false;
      
      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom",1, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
      sub_opp_odom = this->create_subscription<nav_msgs::msg::Odometry>("/opp_racecar/odom",1, std::bind(&MPCNode::oppOdomCallback, this, std::placeholders::_1));
      sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",1, std::bind(&MPCNode::mapCallback, this, std::placeholders::_1));
      pub_nav = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive",1000);
      pub_points = this->create_publisher<visualization_msgs::msg::MarkerArray>("/current_pose",1000);
      pub_monte_carlo_predictions = this->create_publisher<visualization_msgs::msg::MarkerArray>("/monte_carlo_predictions", 1);
      file_trace.open("./ego_vehicle_trace_K_216_N_20.csv", std::fstream::out | std::fstream::trunc);
      if(file_trace.fail())
      	perror("./ego_vehicle_trace.csv");
      file_trace << "#x_ego, y_ego, x_opp, y_opp\n";
      file_timings.open("./mpc_mc_timings_K_216_N_20.csv", std::fstream::out | std::fstream::trunc);
      if(file_timings.fail())
      	perror("./mpc_mc_timings.csv");
      file_timings << "#mpc, monte_carlo\n";
   }
   ~MPCNode(){
   	file_trace.close();
   	file_timings.close();
   }
private:
	
   std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;
   tf2::Stamped<tf2::Transform> tf_base_link_to_map;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_opp_odom;
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
   rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_nav;
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_points;
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_monte_carlo_predictions;
   
   std::ofstream file_trace;
   std::ofstream file_timings;
   std::mt19937 gen;
   std::uniform_real_distribution<> velocity_dist;
   std::uniform_real_distribution<> steering_angle_dist;
   nav_msgs::msg::Odometry last_opp_odom_msg;
   bool received_opp_odom;
   
   MPC mpc_controller;
   Trajectory ref_trajectory;
   float velocity;
   float steering_angle;
   bool init_front_left_wheel;
   bool init_front_right_wheel;
   
   float getRadians(int degrees){
   	return degrees * 0.0175433;
   }
   
   void visualizePoints(std::vector<geometry_msgs::msg::Point>& points){
   	visualization_msgs::msg::Marker p1;
   	visualization_msgs::msg::MarkerArray point_array;
	p1.header.frame_id = "map";
	p1.header.stamp = this->get_clock()->now();
	p1.ns = "mpc points";
	p1.id = 0;
	p1.type = visualization_msgs::msg::Marker::SPHERE;
  	p1.action = visualization_msgs::msg::Marker::ADD;
	p1.pose.orientation.x = 0.0;
	p1.pose.orientation.y = 0.0;
	p1.pose.orientation.z = 0.0;
	p1.pose.orientation.w = 1.0;
	p1.scale.x = p1.scale.y = p1.scale.z = .2;
	p1.color.a = 1.0;
	p1.color.r = 0.0;
	p1.color.g = 1.0;
	p1.color.b = 0.0;
	
	int current_id = 1;
	for(geometry_msgs::msg::Point p : points){
	p1.id = current_id;
	p1.pose.position.x = p.x;
  	p1.pose.position.y = p.y;
  	p1.pose.position.z = p.z;
	point_array.markers.push_back(p1);
	current_id++;
	}
	pub_points->publish(point_array);
   }
   
   void visualizeLines(std::vector<State>& states, int id){
	visualization_msgs::msg::Marker point;
	visualization_msgs::msg::MarkerArray point_array;
	point.header.frame_id = "map";
	point.header.stamp = this->get_clock()->now();
	point.ns = "gap constraints";
	point.id = id;
  	point.action = visualization_msgs::msg::Marker::ADD;
	point.pose.position.z = 0.0;
	point.pose.orientation.x = 0.0;
	point.pose.orientation.y = 0.0;
	point.pose.orientation.z = 0.0;
	point.pose.orientation.w = 1.0;
	point.type = visualization_msgs::msg::Marker::LINE_STRIP;
	point.scale.x = .05;
	point.color.a = 1.0;
	if(id == 0){
		point.color.r = 1.0;
		point.color.g = 0.0;
		point.color.b = 1.0;
	}
	else if(id==1){
		point.type = visualization_msgs::msg::Marker::LINE_LIST;
		point.color.r = 1.0;
		point.color.g = 0.0;
		point.color.b = 0.0;
	}
	else{
		point.color.r = 0.0;
		point.color.g = 1.0;
		point.color.b = 1.0;
	}
	
	geometry_msgs::msg::Point pt;
	for(State s: states){
		pt.x = s.getX();
		pt.y = s.getY();
		pt.z = 0.2;
		point.points.push_back(pt);
	}
	
	point_array.markers.push_back(point);
	pub_points->publish(point_array);
   }
   
   void visualizePredictions(State trajectories[NUM_TRAJECTORIES][TRAJECTORY_LENGTH+1]){
	visualization_msgs::msg::Marker tree_lines;
	visualization_msgs::msg::MarkerArray tree_lines_array;
	tree_lines.header.frame_id = "map";
	tree_lines.header.stamp = this->get_clock()->now();
	tree_lines.ns = "monte carlo trajectories";
  	tree_lines.action = visualization_msgs::msg::Marker::ADD;
	tree_lines.pose.position.z = 0.0;
	tree_lines.pose.orientation.x = 0.0;
	tree_lines.pose.orientation.y = 0.0;
	tree_lines.pose.orientation.z = 0.0;
	tree_lines.pose.orientation.w = 1.0;
	tree_lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
	tree_lines.scale.x = .05;
	tree_lines.color.a = 1.0;
	tree_lines.color.r = 0.0;
	tree_lines.color.g = 0.0;
	tree_lines.color.b = 1.0;
	
	geometry_msgs::msg::Point pt;
	for(int i = 0; i < NUM_TRAJECTORIES; i++){
		tree_lines.points.clear();
		tree_lines.id = i;
		for(int j = 0; j < TRAJECTORY_LENGTH+1; j++){
			pt.x = trajectories[i][j].getX();
			pt.y = trajectories[i][j].getY();
			pt.z = 0.2;
			tree_lines.points.push_back(pt);
		}
		tree_lines_array.markers.push_back(tree_lines);
	}
	pub_monte_carlo_predictions->publish(tree_lines_array);
   }
   
   void recordPoints(std::vector<State>& mpc_predict_trajectory, State monte_carlo_trajectories[NUM_TRAJECTORIES][TRAJECTORY_LENGTH+1]){
   	file_trace << mpc_predict_trajectory[0].getX() << "," << mpc_predict_trajectory[0].getY() 
				<< "," << monte_carlo_trajectories[0][0].getX() << "," << monte_carlo_trajectories[0][0].getY() << ",|,";
   	for(int i = 0; i < NUM_TRAJECTORIES; i++){
		for(int j = 1; j < TRAJECTORY_LENGTH+1; j++){
			file_trace << mpc_predict_trajectory[j].getX() << "," << mpc_predict_trajectory[j].getY() 
				<< "," << monte_carlo_trajectories[i][j].getX() << "," << monte_carlo_trajectories[i][j].getY() << ",";
		}
		file_trace << "|,";
	}
	file_trace << "\n";
   }
   
   void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg){
   	geometry_msgs::msg::TransformStamped tf;
    	
	try{
	tf = tf_buffer->lookupTransform("map", "ego_racecar/base_link", tf2::TimePointZero);
	tf2::fromMsg(tf, tf_base_link_to_map) ;
	} 
	catch(const tf2::TransformException &ex){
	RCLCPP_INFO(this->get_logger(), "Could NOT transform base link to map: %s", ex.what());
	return;
	}
	
	tf2::Vector3 center_of_mass(.17145, 0, 0);
	tf2::Vector3 center_of_mass_map = tf_base_link_to_map * center_of_mass;
	
	double roll, pitch, yaw;
    	tf2::Matrix3x3(tf_base_link_to_map.getRotation()).getRPY(roll, pitch, yaw);
	
	State curr_state(center_of_mass_map.getX(), center_of_mass_map.getY(), yaw);
	Input curr_input((double)velocity, (double)steering_angle);
	
	std::vector<State> reference_trajectory = ref_trajectory.findRefTrajectoryPurePursuit(curr_state, TRAJECTORY_LENGTH+1);
	visualizeLines(reference_trajectory, 0);
	
	auto mpc_start = std::chrono::high_resolution_clock::now();
	if(mpc_controller.UpdateMPC(curr_state, curr_input, reference_trajectory)){
		std::vector<State> mpc_predict_trajectory = mpc_controller.getPredictedTrajectory();
		auto mpc_stop = std::chrono::high_resolution_clock::now();
		visualizeLines(mpc_predict_trajectory, 2);
		Input command = mpc_controller.getNextInput();
		velocity = command.getVelocity();
		steering_angle = command.getSteeringAngle();
		
		ackermann_msgs::msg::AckermannDriveStamped drive_msg;
		drive_msg.header.stamp = this->get_clock()->now();
		drive_msg.header.frame_id = "laser";
		drive_msg.drive.speed = velocity;
		drive_msg.drive.steering_angle = steering_angle;
	      	pub_nav->publish(drive_msg);
	      	
	      	if(received_opp_odom){
	      		auto mc_start = std::chrono::high_resolution_clock::now();
		      	tf2::Quaternion q(last_opp_odom_msg.pose.pose.orientation.x, 
	      			  last_opp_odom_msg.pose.pose.orientation.y, 
	      			  last_opp_odom_msg.pose.pose.orientation.z, 
	      			  last_opp_odom_msg.pose.pose.orientation.w);
		      	double roll, pitch, yaw;
		      	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
		      	Model vehicle_model;
		      	State monte_carlo_trajectories[NUM_TRAJECTORIES][TRAJECTORY_LENGTH+1];
		      	for (int i = 0; i < NUM_TRAJECTORIES; i++){
		    	    State curr_state(last_opp_odom_msg.pose.pose.position.x, last_opp_odom_msg.pose.pose.position.y, yaw);
		    	    double random_velocity = sqrt(pow(last_opp_odom_msg.twist.twist.linear.x,2)+pow(last_opp_odom_msg.twist.twist.linear.y,2));
		    	    double random_steering_angle = steering_angle_dist(gen);
		    	    monte_carlo_trajectories[i][0] = curr_state;
			    for (int j = 1; j < TRAJECTORY_LENGTH+1; j++){
			    	if(j%5 == 0){
			    		random_velocity = velocity_dist(gen);
			    		random_steering_angle = steering_angle_dist(gen);
				}
				Input curr_input(random_velocity, random_steering_angle);
				State next_state = vehicle_model.get_next_state(curr_state, curr_input);
				curr_state = next_state;
				monte_carlo_trajectories[i][j] = next_state;
			    }
		      	}
		      	auto mc_stop = std::chrono::high_resolution_clock::now();
		      	file_timings << std::chrono::duration_cast<std::chrono::microseconds>(mpc_stop-mpc_start).count() << ","
		      	             << std::chrono::duration_cast<std::chrono::microseconds>(mc_stop-mc_start).count() << "\n";
		      	recordPoints(mpc_predict_trajectory, monte_carlo_trajectories);
			visualizePredictions(monte_carlo_trajectories);
		}
      	}
      	else{
      		ackermann_msgs::msg::AckermannDriveStamped drive_msg;
		drive_msg.header.stamp = this->get_clock()->now();
		drive_msg.header.frame_id = "laser";
		drive_msg.drive.speed = 0.0;
		drive_msg.drive.steering_angle = 0.0;
	      	pub_nav->publish(drive_msg);
      	}
   }
   
   void oppOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
   	last_opp_odom_msg = *odom_msg;
   	received_opp_odom = true;
   }
   
   void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg){
   	ref_trajectory.setGlobalMap(*map_msg);
   	std::cout << "Passed in map" << std::endl;
   }
};
int main(int argc, char **argv){
   printf("hello world mpc package running\n");
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<MPCNode>());
   rclcpp::shutdown();
   return 0;
}
