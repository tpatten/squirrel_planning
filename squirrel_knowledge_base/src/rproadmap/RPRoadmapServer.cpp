#include "ros/ros.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 
#include "squirrel_planning_knowledge_msgs/KnowledgeItem.h"
#include "squirrel_knowledge_base/RPRoadmapServer.h"
#include "mongodb_store/message_store.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "diagnostic_msgs/KeyValue.h"

namespace KCL_rosplan {

	/* constructor */
	RPRoadmapServer::RPRoadmapServer(ros::NodeHandle &nh, std::string &dp)
	 : message_store(nh), dataPath(dp) {
		add_knowledge_pub = nh.advertise<squirrel_planning_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/add_knowledge", 10, true);
		remove_knowledge_pub = nh.advertise<squirrel_planning_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/remove_knowledge", 10, true);
		map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
	}

	/**
	 * Generates waypoints and stores them in the knowledge base and scene database
	 */
	bool RPRoadmapServer::generateRoadmap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		// clear previous roadmap from knowledge base
		ROS_INFO("KCL: (RPRoadmapServer) Cleaning old roadmap");
		squirrel_planning_knowledge_msgs::KnowledgeItem clearWP;
		clearWP.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE;
		clearWP.instance_type = "Waypoint";
		remove_knowledge_pub.publish(clearWP);

		squirrel_planning_knowledge_msgs::KnowledgeItem clearConn;
		clearConn.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		clearConn.attribute_name = "connected";
		remove_knowledge_pub.publish(clearConn);

		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
			message_store.deleteID(db_name_map[wit->first]);
		db_name_map.clear();
 
		// read map
		ROS_INFO("KCL: (RPRoadmapServer) Reading in map");
		nav_msgs::GetMap mapSrv;
		map_client.call(mapSrv);
		nav_msgs::OccupancyGrid map = mapSrv.response.map;

		// generate waypoints
		ROS_INFO("KCL: (RPRoadmapServer) Generating roadmap");
		createPRM(map, 8, 40, 100);

		// print map for debugging
		std::ofstream wpsFile;
		wpsFile.open("/home/michael/WPs.txt");
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
		for (std::vector<std::string>::iterator nit=wit->second.neighbours.begin(); nit!=wit->second.neighbours.end(); ++nit)
			wpsFile << wit->second.x << " " << wit->second.y << " " << waypoints[*nit].x << " " << waypoints[*nit].y << std::endl;
		wpsFile.close();

		// add roadmap to knowledge base and scene database
		ROS_INFO("KCL: (RPRoadmapServer) Adding knowledge");
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {

			// instance
			squirrel_planning_knowledge_msgs::KnowledgeItem addWP;
			addWP.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE;
			addWP.instance_type = "Waypoint";
			addWP.instance_name = wit->first;
			add_knowledge_pub.publish(addWP);

			// predicates
			for (std::vector<std::string>::iterator nit=wit->second.neighbours.begin(); nit!=wit->second.neighbours.end(); ++nit) {
				squirrel_planning_knowledge_msgs::KnowledgeItem addConn;
				addConn.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				addConn.attribute_name = "connected";
				diagnostic_msgs::KeyValue pairFrom;
				pairFrom.key = "from";
				pairFrom.value = wit->first;
				addConn.values.push_back(pairFrom);
				diagnostic_msgs::KeyValue pairTo;
				pairTo.key = "to";
				pairTo.value = *nit;
				addConn.values.push_back(pairTo);
				add_knowledge_pub.publish(addConn);			
			}

			//data
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = map.header.frame_id;
			pose.pose.position.x = wit->second.x;
			pose.pose.position.y = wit->second.y;
			pose.pose.position.z = 0.0;
			std::string id(message_store.insertNamed(wit->first, pose));
			db_name_map[wit->first] = id;
		}

		ROS_INFO("KCL: (RPRoadmapServer) Done");
		return true;
	}

	/*
	 * Input:
	 * 	K, number of seed waypoints
	 * 	D, distance of random motions
	 * 	R, radius of random connections
	 * Output: A roadmap G = (V, E)
	*/
	void RPRoadmapServer::createPRM(nav_msgs::OccupancyGrid map, unsigned int K, unsigned int D, unsigned int R) {

		// map info
		int width = map.info.width;
		int height = map.info.height;

		if(width==0 || height==0) {
			ROS_INFO("KCL: (RPRoadmapServer) Empty map");
			return;
		}

		// V <-- empty set; E <-- empty set.
		waypoints.clear();
		edges.clear();

		// while cardinality(V) < K do
		while(waypoints.size() < K) {

			// sample collision-free configuration at random
			int x = rand() % width;
			int y = rand() % height;

			if(map.data[ (width*y + x) ] <= 0) {
				std::stringstream ss;
				ss << "wp" << waypoints.size();
				Waypoint wp(ss.str(), x, y);
				waypoints[wp.wpID] = wp;
			}
		}

		while(!allConnected() && waypoints.size()<50) {

			for(size_t grow=0; grow<3; grow++) {
				// get random free point
				int x = rand() % width;
				int y = rand() % height;
				while(map.data[ (width*y + x) ] > 0) {
					x = rand() % width;
					y = rand() % height;
				}

				// find closest waypoint from V
				double dist = sqrt((width*width)+(height*height));
				Waypoint closest;
				for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
					Waypoint w = wit->second;
					double d = sqrt( ((x - w.x)*(x - w.x)) + ((y - w.y)*(y - w.y)) );
					if(d < dist) {
						closest = w;
						dist = d;
					}
				}
				if(closest.wpID.compare("wp_err")==0) {
					ROS_INFO("KCL: (RPRoadmapServer) Error in RRT (can't find closest neighbour)");
					return;
				}

				// new point
				int xNew = closest.x + (int)(D*(x-closest.x)/dist);
				int yNew = closest.y + (int)(D*(y-closest.y)/dist);
			
				// (TODO check collision and) add to waypoints
				std::stringstream ss;
				ss << "wp" << waypoints.size();
				Waypoint wpNew(ss.str(), xNew, yNew);
				waypoints[wpNew.wpID] = wpNew;
				closest.neighbours.push_back(wpNew.wpID);
				wpNew.neighbours.push_back(closest.wpID);
				Edge e(closest.wpID, wpNew.wpID);
				edges.push_back(e);

				// try and connect things up seeds to map
				makeConnections(R);

			}
		}
	}

	/* returns true if waypoints form a single connected component */
	bool RPRoadmapServer::allConnected() {

		if(waypoints.size()<1) return true;

		std::map<std::string,bool> connected;
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
			connected[wit->first] = false;

		connectRecurse(connected, waypoints["wp0"]);
		
		int count = 0;
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit)
			if(connected[wit->first]) count++;
// std::cout << count << " " << waypoints.size() << std::endl;

		return (count == waypoints.size());
	}

	/* recursive call from allConnected */
	void RPRoadmapServer::connectRecurse(std::map<std::string,bool> &connected, Waypoint &waypoint) {
		if(connected[waypoint.wpID]) return;
		connected[waypoint.wpID] = true;
		for(size_t i=0; i<waypoint.neighbours.size(); i++)
			if(!connected[waypoints[waypoint.neighbours[i]].wpID])
				connectRecurse(connected, waypoints[waypoint.neighbours[i]]);
	}

	/* attempts to make new connections */
	bool RPRoadmapServer::makeConnections(unsigned int R) {

		bool newEdges = false;
		for (std::map<std::string,Waypoint>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
		for (std::map<std::string,Waypoint>::iterator sit=waypoints.begin(); sit!=waypoints.end(); ++sit) {
			if(wit->first.compare(sit->first)==0 || find(wit->second.neighbours.begin(), wit->second.neighbours.end(), sit->first) != wit->second.neighbours.end())
				continue;
			Waypoint w = wit->second;
			Waypoint s = sit->second;
			double d = sqrt( ((s.x - w.x)*(s.x - w.x)) + ((s.y - w.y)*(s.y - w.y)) );
			if(d<R) {
				wit->second.neighbours.push_back(s.wpID);
				sit->second.neighbours.push_back(w.wpID);
				Edge e(w.wpID, s.wpID);
				edges.push_back(e);
				newEdges = true;
			}
		}};
		return newEdges;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_roadmap_server");
		ros::NodeHandle nh("~");

		// default config
		std::string dataPath = "common/";
		nh.param("data_path", dataPath, dataPath);
		
		// init services
		KCL_rosplan::RPRoadmapServer rms(nh, dataPath);
		ros::ServiceServer service = nh.advertiseService("/kcl_rosplan/roadmap_server", &KCL_rosplan::RPRoadmapServer::generateRoadmap, &rms);

		ROS_INFO("KCL: (RPRoadmapServer) Ready to receive");
		ros::spin();
		return 0;
	}
