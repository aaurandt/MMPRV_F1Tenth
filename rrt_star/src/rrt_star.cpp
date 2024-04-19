// RRT in Section 3: https://arxiv.org/pdf/1105.1186.pdf
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include "goal_point_locator.hpp"
#include <random>

class RRTStar : public rclcpp::Node{
public:
	RRTStar() : Node("rrt_star") {
		lookahead_dist = 4.5;
   		max_expansion_dist = .3;
   		near_radius = 1.0;
   		goal_threshold = 0.25;
   		num_iterations = 1000;
   		num_occ_grid_updates = 0;
   		init_laser_to_map_tf = false;
   		active = false;
   		
   		// initialize random number generation
   		std::random_device rd;
   		gen = std::mt19937(rd());
   		x_dist = std::uniform_real_distribution<double>(0.0, lookahead_dist);
   		y_dist = std::uniform_real_distribution<double>(-1*lookahead_dist, lookahead_dist);
   		
   		tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      		tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
   		
		sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/opp_racecar/odom",1, std::bind(&RRTStar::odom_callback, this, std::placeholders::_1));
		sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/opp_scan", 10, std::bind(&RRTStar::scan_callback, this, std::placeholders::_1));
		sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 1, std::bind(&RRTStar::map_callback, this, std::placeholders::_1));
		pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/opp_drive",1000);
		pub_occ_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occ_grid",1);
		pub_path = this->create_publisher<nav_msgs::msg::Path>("/path",1);
		pub_tree_nodes = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tree_nodes_array", 1);
		pub_tree_lines = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tree_lines_array", 1);
		pub_waypoint = this->create_publisher<visualization_msgs::msg::Marker>("/waypoints", 1);
		
	}
private:
   	std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
   	tf2::Stamped<tf2::Transform> tf_laser_to_map;
   
   	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
   	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
   	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
   	
   	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
   	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occ_grid;
   	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
   	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tree_nodes;
   	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tree_lines;
   	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_waypoint;
   
	double lookahead_dist;
	int inflation_radius;
	double max_expansion_dist;
	double near_radius;
	double goal_threshold;
	GoalPoint goal_point_locator;
	int num_iterations;
	bool init_laser_to_map_tf;
	double input_map_yaw;
	int num_occ_grid_updates;
    	nav_msgs::msg::OccupancyGrid static_map;
    	nav_msgs::msg::OccupancyGrid occ_grid;
	std::mt19937 gen;
	std::uniform_real_distribution<> x_dist;
	std::uniform_real_distribution<> y_dist;
	Position goal;
	bool active;
	
	typedef struct TreeNode {
	    double x, y;
	    double cost; // only used for RRT*
	    int parent; // index of parent node in the tree vector (-1 if root)
	} TreeNode;
	
	std::vector<TreeNode> curr_drive_path;
   
   	// Capturing map
	void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_msg) {
		tf2::Quaternion q(grid_msg->info.origin.orientation.x, 
		   	grid_msg->info.origin.orientation.y, 
		   	grid_msg->info.origin.orientation.z, 
	   		grid_msg->info.origin.orientation.w);
		double roll, pitch;
	    	tf2::Matrix3x3(q).getRPY(roll, pitch, input_map_yaw);

		inflation_radius = 3;
		static_map = *grid_msg;
		int index;
		for(int i = 0; i < grid_msg->info.width; i++){
			for(int j = 0; j < grid_msg->info.height; j++){
				index = grid_msg->info.width * j + i;
				if(grid_msg->data[index] == 100){
					std::vector<int> indices = get_inflated_row_major_indices(i,j);
					for(int expanded_index : indices){
						static_map.data[expanded_index] = 100;
					}
				}
			}
		}
		//inflation_radius = 5;
		occ_grid = static_map;
		active = true;
	}
	
   	//RRT main loop
	void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
	    geometry_msgs::msg::TransformStamped tf;
	    try{
			tf = tf_buffer->lookupTransform("map", "opp_racecar/laser", tf2::TimePointZero);
			tf2::fromMsg(tf, tf_laser_to_map) ;
	    } 
	    catch(const tf2::TransformException &ex){
	  		RCLCPP_INFO(this->get_logger(), "Could NOT transform laser to map: %s", ex.what());
	  		return;
	    }

	    if(active){
		    // tree as std::vector
		    tf2::Quaternion q(odom_msg->pose.pose.orientation.x, 
                           odom_msg->pose.pose.orientation.y, 
                           odom_msg->pose.pose.orientation.z, 
                           odom_msg->pose.pose.orientation.w);
		    double roll, pitch, yaw;
		    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
		
		    State curr_state(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, yaw);
		    
		    goal = goal_point_locator.findRefGoalPointPurePursuit(curr_state);
		    std::vector<TreeNode> tree;
		    TreeNode start_node;
		    start_node.x = odom_msg->pose.pose.position.x;
		    start_node.y = odom_msg->pose.pose.position.y;
		    start_node.parent = -1;
		    tree.emplace_back(start_node);
		    for(int i = 0; i < num_iterations; i++){
			std::vector<double> sampled_point = sample();
			int nearest_point = nearest(tree, sampled_point);
			TreeNode new_node = steer(tree[nearest_point], sampled_point);
			if(!check_collision(tree[nearest_point], new_node)){
				new_node.parent = nearest_point;  //nearest,new
				new_node.cost = cost(tree, new_node);

				std::vector<int> near_neighbors = near(tree, new_node);
				for(int node_index : near_neighbors){ //Find best neighbor
					if(!check_collision(tree[node_index], new_node)){
						double cost = tree[node_index].cost + line_cost(tree[node_index], new_node);
						if(cost < new_node.cost){
							new_node.parent = node_index;
							new_node.cost = cost;
						}
					}
				}

				for(int node_index : near_neighbors){ //Rewire the tree
					if(!check_collision(new_node, tree[node_index])){
						double cost = new_node.cost + line_cost(new_node, tree[node_index]);
						if(cost < tree[node_index].cost){
							tree[node_index].cost = cost;
							tree[node_index].parent = tree.size();
						}
					}
				}

				tree.emplace_back(new_node);
				if(is_goal(new_node, goal.getX(), goal.getY())){
		 			curr_drive_path = find_path(tree, new_node);
					nav_msgs::msg::Path path_to_publish;
					path_to_publish.header.stamp = this->get_clock()->now();
					path_to_publish.header.frame_id = "map";
					for(int i = 0; i < curr_drive_path.size(); i++){
						geometry_msgs::msg::PoseStamped point;
						point.pose.position.x = curr_drive_path[i].x;
						point.pose.position.y = curr_drive_path[i].y;
						path_to_publish.poses.push_back(point);
					}
					pub_path->publish(path_to_publish);
					visualize_tree(tree, new_node);
					break;
				}
			}
		    }
		    drive_along_path(curr_state);
	    }
	}
	
	// Returns a sampled point on the map in free space
	std::vector<double> sample() {
	    tf2::Vector3 map_point;
	    int grid_x, grid_y;
	    do{
	    tf2::Vector3 laser_point(x_dist(gen), y_dist(gen), 0.0);
	    map_point = tf_laser_to_map * laser_point;
	    get_xy_from_map_to_grid(map_point.x(), map_point.y(), &grid_x, &grid_y);
	    }while(occ_grid.data[occ_grid.info.width * grid_y + grid_x] != 0);
	   
	    return {map_point.x(), map_point.y(), map_point.z()};
	}
	
	std::vector<int> get_inflated_row_major_indices(double x, double y){
	    std::vector<int> indices;
	    int index;
	    for(int i = (-1*inflation_radius+x); i <= (inflation_radius+x); i++){
		for(int j = (-1*inflation_radius+y); j <= (inflation_radius+y); j++){
			index = static_map.info.width * j + i;
			if(index >= 0 && index < (static_map.info.width * static_map.info.height))
				indices.emplace_back(index);
		}
	    }
	    return indices;
	}

	void get_xy_from_map_to_grid(double map_x, double map_y, int* grid_x, int* grid_y){
		double x = map_x - occ_grid.info.origin.position.x;
		double y = map_y - occ_grid.info.origin.position.y;
		*grid_x = static_cast<int>((x*cos(input_map_yaw)+y*sin(input_map_yaw))/occ_grid.info.resolution) - 1;
	     	*grid_y = static_cast<int>((-1*x*sin(input_map_yaw)+y*cos(input_map_yaw))/occ_grid.info.resolution) - 1;
	}

	void visualize_tree(std::vector<TreeNode>& tree, TreeNode goal_node){
		visualization_msgs::msg::Marker tree_nodes, tree_lines;
		visualization_msgs::msg::MarkerArray tree_nodes_array, tree_lines_array;
		tree_nodes.header.frame_id = tree_lines.header.frame_id = "map";
		tree_nodes.header.stamp = tree_lines.header.stamp = this->get_clock()->now();
		tree_nodes.ns = "rrt nodes"; tree_lines.ns = "rrt lines";
		tree_nodes.id = tree_lines.id = 0;
	  	tree_nodes.action = tree_lines.action = visualization_msgs::msg::Marker::ADD;
		tree_nodes.pose.position.z = tree_lines.pose.position.z = 0.0;
		tree_nodes.pose.orientation.x = tree_lines.pose.orientation.x = 0.0;
		tree_nodes.pose.orientation.y = tree_lines.pose.orientation.y = 0.0;
		tree_nodes.pose.orientation.z = tree_lines.pose.orientation.z = 0.0;
		tree_nodes.pose.orientation.w = tree_lines.pose.orientation.w = 1.0;
		tree_nodes.type = visualization_msgs::msg::Marker::POINTS;
		tree_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
		tree_nodes.scale.x = tree_nodes.scale.y = tree_nodes.scale.z = .05;
		tree_lines.scale.x = .01;
		tree_nodes.color.a = tree_lines.color.a = 1.0;
		tree_nodes.color.r = tree_lines.color.r = 0.0;
		tree_nodes.color.g = tree_lines.color.g = 0.0;
		tree_nodes.color.b = tree_lines.color.b = 1.0;

		geometry_msgs::msg::Point pt, pt_parent;
		for(TreeNode n : tree){
			pt.x = n.x;
			pt.y = n.y;
			tree_nodes.points.push_back(pt);
			if(n.parent != -1){
				pt_parent.x = tree[n.parent].x;
				pt_parent.y = tree[n.parent].y;
				tree_lines.points.push_back(pt);
				tree_lines.points.push_back(pt_parent);
			}
		}
		
		tree_nodes_array.markers.push_back(tree_nodes);
		tree_lines_array.markers.push_back(tree_lines);

		tree_nodes.color.r = tree_lines.color.r = 1.0;
		tree_nodes.color.b = tree_lines.color.b = 0.0;
		tree_nodes.scale.x = tree_nodes.scale.y = tree_nodes.scale.z = .1;
		tree_lines.scale.x = .05;
		tree_nodes.points.clear();
		tree_lines.points.clear();
		tree_nodes.id = tree_lines.id = 1;

		std::vector<TreeNode> path = find_path(tree, goal_node);
		for(TreeNode n : path){
			pt.x = n.x;
			pt.y = n.y;
			tree_nodes.points.push_back(pt);
			if(n.parent != -1){
				pt_parent.x = tree[n.parent].x;
				pt_parent.y = tree[n.parent].y;
				tree_lines.points.push_back(pt);
				tree_lines.points.push_back(pt_parent);
			}
		}

		tree_nodes_array.markers.push_back(tree_nodes);
		tree_lines_array.markers.push_back(tree_lines);
		
		pub_tree_nodes->publish(tree_nodes_array);
		pub_tree_lines->publish(tree_lines_array);
	}
	
	//Returns the nearest node to the sampled point
	int nearest(std::vector<TreeNode> &tree, std::vector<double> &sampled_point) {
	    int nearest_node = 0;
	    double smallest_distance = 1000000;
	    double current_distance;
	    double x, y;
	    for(int i = 0; i < tree.size(); i++){
		x = sampled_point[0] - tree[i].x;
		y = sampled_point[1] - tree[i].y;
	    	current_distance = sqrt(x * x + y * y);
		if(current_distance < smallest_distance){
			smallest_distance = current_distance;
			nearest_node = i;
		}
	    }
	    return nearest_node;
	}

	//Expands the tree towards the sample point within a max distance (creates new node)
	TreeNode steer(TreeNode &nearest_node, std::vector<double> &sampled_point) {
	    TreeNode new_node;
	    double x = sampled_point[0] - nearest_node.x;
	    double y = sampled_point[1] - nearest_node.y;
	    double distance = sqrt(x * x + y * y);
	    if(distance < max_expansion_dist){
		new_node.x = sampled_point[0];
		new_node.y = sampled_point[1];
	    }
	    else{
		new_node.x = cos(atan2(y, x)) * max_expansion_dist + nearest_node.x;
		new_node.y = sin(atan2(y, x)) * max_expansion_dist + nearest_node.y;
	    }

	    return new_node;
	}

	//Checks if the path between the nearest node and new node is collision free
	bool check_collision(TreeNode &nearest_node, TreeNode &new_node) {
	    double x = new_node.x - nearest_node.x;
	    double y = new_node.y - nearest_node.y;
	    double angle = atan2(y, x);
	    double distance = sqrt(x * x + y * y);
	    double curr_x = nearest_node.x;
	    double curr_y = nearest_node.y;
	    int grid_x, grid_y;
	    
	    double i = 0;
	    while(i <= distance){
		curr_x += cos(angle)*i;
		curr_y += sin(angle)*i;
		i += occ_grid.info.resolution;
		if(curr_x > new_node.x)
			curr_x = new_node.x;
		if(curr_y > new_node.y)
			curr_y = new_node.y;
	    	get_xy_from_map_to_grid(curr_x, curr_y, &grid_x, &grid_y);
		int index = occ_grid.info.width * grid_y + grid_x;
		if(index >= 0 && index < (occ_grid.info.width*occ_grid.info.height)){
	    		if(occ_grid.data[index] == 100)
				return true;
		}
	    }
	    return false;
	}

	//Checks if the node is close enough to the goal
	bool is_goal(TreeNode &latest_added_node, double goal_x, double goal_y) {
	    double x = latest_added_node.x - goal_x;
	    double y = latest_added_node.y - goal_y;
	    double distance = sqrt(x * x + y * y);
	    return distance <= goal_threshold;
	}

	//Traverses the tree from the node that has been determined as goal and returns as path
	std::vector<TreeNode> find_path(std::vector<TreeNode> &tree, TreeNode &latest_added_node) {    
	    std::vector<TreeNode> found_path;
	    TreeNode current_node = latest_added_node;
	    while(current_node.parent != -1){
		found_path.emplace_back(current_node);
	    	current_node = tree[current_node.parent];
	    }
	    return found_path;
	}

	//Calculates cost associated with a node
	double cost(std::vector<TreeNode> &tree, TreeNode &node) {
	    if(node.parent == -1)
		return 0;
	    else
	    	return tree[node.parent].cost + line_cost(tree[node.parent], node);
	}

	//Calculates the cost of the straight line path between two nodes
	double line_cost(TreeNode &n1, TreeNode &n2) {
	    double x = n1.x - n2.x;
	    double y = n1.y - n2.y;
	    return sqrt(x * x + y * y);
	}

	//Finds the set the nodes in the neighborhood of a node
	std::vector<int> near(std::vector<TreeNode> &tree, TreeNode &node) {
	    std::vector<int> neighborhood;
	    double x, y, distance;
	    for(int i = 0; i < tree.size(); i++){
		x = tree[i].x - node.x;
		y = tree[i].y - node.y;
		distance = sqrt(x * x + y * y);
		if(distance <= near_radius)
			neighborhood.emplace_back(i);
	    }

	    return neighborhood;
	}
	
	void drive_along_path(State curr_state){
		if(curr_drive_path.size() > 0) {
			double distance, angle, diff_angle;
			double drive_lookahead_dist = 1.0;
			TreeNode curr_drive_waypoint = curr_drive_path[0];
			for(TreeNode curr_pose : curr_drive_path){
				distance = sqrt(pow(curr_pose.x - curr_state.getX(), 2) + 
						pow(curr_pose.y - curr_state.getY(), 2));
				angle = atan2(curr_pose.y - curr_state.getY(), curr_pose.x - curr_state.getX());
				diff_angle = curr_state.getYaw() - angle;
				// Make diff_angle between -180 and 180 degrees
				if(diff_angle < (-1*M_PI))
					diff_angle += 2*M_PI;
				else if(diff_angle > M_PI)
					diff_angle -= 2*M_PI;
				if((diff_angle < 2.35619) && (diff_angle > -2.35619)){ // Waypoint is not behind car (+-135 degree window)
					if(distance <= drive_lookahead_dist){
						curr_drive_waypoint = curr_pose;
						break;
					}		
				}
			}
			// Transform goal point to vehicle frame of reference
			double transformed_waypoint_y = -1*(curr_drive_waypoint.x - curr_state.getX())*sin(curr_state.getYaw()) 
							+ (curr_drive_waypoint.y - curr_state.getY())*cos(curr_state.getYaw());

			// Calculate curvature/steering angle
			double steering_angle = 2 * transformed_waypoint_y / pow(drive_lookahead_dist, 2);

			// Publish drive message
			ackermann_msgs::msg::AckermannDriveStamped drive_msg;
			drive_msg.header.stamp = this->get_clock()->now();
			drive_msg.header.frame_id = "laser";

	   		steering_angle = steering_angle * 0.25;

			// Max out steering angle to 25 degrees
			if(steering_angle < -.4189){ // 25 degrees in radians
				steering_angle = -.4189;
			}
			else if(steering_angle > .4189){
				steering_angle = .4189;
			}
			
			drive_msg.drive.steering_angle = steering_angle;
			if(steering_angle > -.087267 && steering_angle < .087267) // 5 degrees in radians
	 			drive_msg.drive.speed = 5.0;
	 		else if (steering_angle > -.261799 && steering_angle < .261799) // 15 degrees in radians
	 			drive_msg.drive.speed = 3.5;
			else
	 			drive_msg.drive.speed = 1.0;
	      		pub_drive->publish(drive_msg);
      		}
	}
	
	// Updates Occupancy Grid
	void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
	    if(num_occ_grid_updates >= 100) //Reset Occupancy Grid periodically
		occ_grid = static_map;
	    for(int i = 90*3; i < 270*3; i=i+3){ // Only looking at 90 to 270 degree window in front of car
	 	 // Preprocess the ranges array
	 	 double average_dist = (scan_msg->ranges[i] + scan_msg->ranges[i+1] + scan_msg->ranges[i+2])/3.0;
	 	 if(average_dist <= 3.0){ // Objects are less than or equal to 3.0 meters away
			tf2::Vector3 laser_point(-1 * average_dist * cos((i/3)*.0174533), -1 * average_dist * sin((i/3)*.0174533), 0.0); 
			tf2::Vector3 map_point = tf_laser_to_map * laser_point; //Transform from laser frame to map frame
		        int x_index, y_index;
			get_xy_from_map_to_grid(map_point.x(), map_point.y(), &x_index, &y_index); //Transform to occupancy grid data[] coordinates
			int index = occ_grid.info.width * y_index + x_index;
			if(index >= 0 && index < (occ_grid.info.width*occ_grid.info.height)){
				if(occ_grid.data[index] != 100){
					std::vector<int> indices = get_inflated_row_major_indices(x_index,y_index);
					for(int i : indices)
						occ_grid.data[i] = 100;
				}
			}
		 }
	    }
	   occ_grid.header.stamp = this->get_clock()->now();
	   pub_occ_grid->publish(occ_grid);
	   num_occ_grid_updates++;
	}

};
int main(int argc, char **argv){
   printf("hello world rrt star package running\n");
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<RRTStar>());
   rclcpp::shutdown();
   return 0;
}
