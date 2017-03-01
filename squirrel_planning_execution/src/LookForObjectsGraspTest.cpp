#include "ros/ros.h"
#include "squirrel_planning_execution/LookForObjectsGraspTest.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>
#include <algorithm>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "mongodb_store/message_store.h"

#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace std;

namespace Look_Grasp {

    /* constructor */
    LookGraspExecutor::LookGraspExecutor(ros::NodeHandle &nh)
     : node_handle(&nh), message_store(nh) {
     //: message_store(nh), has_received_point_(false) {

        // access to the knowledge base.
        get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
        get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_instances_attributes");
        knowledge_update_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
        filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/mission_filter", 10, true);

        // PRM
        roadmap_service = nh.serviceClient<rosplan_knowledge_msgs::CreatePRM>("/kcl_rosplan/roadmap_server/create_prm");
        add_waypoint_client = nh.serviceClient<rosplan_knowledge_msgs::AddWaypoint>("/kcl_rosplan/roadmap_server/add_waypoint");

        // planner control.
        run_planner_client = nh.serviceClient<std_srvs::Empty>("/kcl_rosplan/planning_server");

        //head_tilt_pub = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10, true);
        //head_nod_pub = nh.advertise<std_msgs::String>("/expression", 10, true);
        //head_down_angle = 0.6;
        //head_up_angle = -0.3;
        //count = 0;

    }

//    void LookGraspExecutor::receivePointLocation(const geometry_msgs::PointStamped::ConstPtr& ptr) {
//        count++;
//        if(count>20) {
//            received_point_ = *ptr;
//            has_received_point_ = true;
//            count = 0;
//        }
//    }

//    /* get point */
//    void LookGraspExecutor::makePoint(std::string& wpID) {

////        // tilt the head kinect up
////        std_msgs::Float64 ht;
////        ht.data = head_up_angle;
////        head_tilt_pub.publish(ht);

//        // Wait for a point to be published.
//        ros::Rate r(10);
//        has_received_point_ = false;
//        while (!has_received_point_ && ros::ok()) {
//            ros::spinOnce();
//            r.sleep();
//        }
//        has_received_point_ = false;
//        ROS_INFO("KCL: (SimpleDemo) Received point");

////        // nod the head
////        std_msgs::String exp;
////        exp.data = "ok";
////        head_nod_pub.publish(exp);
////        ros::Rate nodRate(1);
////        nodRate.sleep();

////        // tilt the head kinect down
////        ht.data = head_down_angle;
////        head_tilt_pub.publish(ht);

//        // convert point to pose
//        geometry_msgs::PoseStamped pose_bl, pose;
//        pose_bl.header.frame_id = "/kinect_depth_optical_frame";
//        pose_bl.pose.position.x = received_point_.point.x;
//        pose_bl.pose.position.y = received_point_.point.y;
//        pose_bl.pose.position.z = received_point_.point.z;
//        tf::Quaternion quat(tf::Vector3(0., 0., 1.), M_PI);
//        pose_bl.pose.orientation.w = quat.w();
//        pose_bl.pose.orientation.x = quat.x();
//        pose_bl.pose.orientation.y = quat.y();
//        pose_bl.pose.orientation.z = quat.z();

//        tf::TransformListener tfl;
//        try {
//            tfl.waitForTransform("/map","/kinect_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
//            tfl.transformPose("/map", pose_bl, pose);
//        } catch ( tf::TransformException& ex ) {
//            ROS_ERROR("%s: error while transforming point", ros::this_node::getName().c_str(), ex.what());
//            return;
//        }

//        pose.pose.orientation.w = quat.w();
//        pose.pose.orientation.x = quat.x();
//        pose.pose.orientation.y = quat.y();
//        pose.pose.orientation.z = quat.z();

//        // set goal pose
//        goal_pose.header.frame_id = pose.header.frame_id;
//        goal_pose.pose.position.x = pose.pose.position.x;
//        goal_pose.pose.position.y = pose.pose.position.y;
//        goal_pose.pose.position.z = pose.pose.position.z;
//        goal_pose.pose.orientation.x = quat.x();
//        goal_pose.pose.orientation.y = quat.y();
//        goal_pose.pose.orientation.z = quat.z();
//        goal_pose.pose.orientation.w = quat.w();

//        // create new waypoint
//        rosplan_knowledge_msgs::AddWaypoint addWPSrv;
//        std::stringstream ss;
//        ss << wpID;
//        addWPSrv.request.id = ss.str();
//        addWPSrv.request.waypoint = pose;
//        addWPSrv.request.connecting_distance = 5;
//        addWPSrv.request.occupancy_threshold = 20;
//        if (!add_waypoint_client.call(addWPSrv)) {
//            ROS_ERROR("KCL: (SimpleDemo) Failed to add a new waypoint for the object");
//        }
//        ROS_INFO("KCL: (SimpleDemo) Road map service returned");
//    }

    void LookGraspExecutor::runDemo() {

        // Add kenny
        rosplan_knowledge_msgs::KnowledgeUpdateService add_kenny;
        add_kenny.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
        rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
        kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
        kenny_knowledge.instance_type = "robot";
        kenny_knowledge.instance_name = "kenny";
        add_kenny.request.knowledge = kenny_knowledge;
        if (!knowledge_update_client.call(add_kenny)) {
            ROS_ERROR("KCL: (LookGrasp) Could not add kenny to the knowledge base.");
            exit(-1);
        }
        ROS_INFO("KCL: (LookGrasp) Added kenny to the knowledge base.");

//        // Add waypoint def
//        rosplan_knowledge_msgs::KnowledgeUpdateService add_wp;
//        add_wp.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
//        rosplan_knowledge_msgs::KnowledgeItem wp_knowledge;
//        wp_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
//        wp_knowledge.instance_type = "waypoint";
//        wp_knowledge.instance_name = "wp0";
//        add_wp.request.knowledge = wp_knowledge;
//        if (!knowledge_update_client.call(add_wp)) {
//            ROS_ERROR("KCL: (LookGrasp) Could not add waypoint to the knowledge base.");
//            exit(-1);
//        }
//        ROS_INFO("KCL: (LookGrasp) Added waypoint to the knowledge base.");

        // clear the old filter
        rosplan_knowledge_msgs::Filter filterMessage;
        filterMessage.function = rosplan_knowledge_msgs::Filter::CLEAR;
        filter_publisher.publish(filterMessage);

        // run once
        ros::spinOnce();
        ROS_INFO("KCL: (LookGrasp) Start the mission");

        // Start by generating some waypoints.
        rosplan_knowledge_msgs::CreatePRM create_prm;
//        create_prm.request.nr_waypoints = 1;
//        create_prm.request.min_distance = 0.5;
//        create_prm.request.casting_distance = 1.6;
//        create_prm.request.connecting_distance = 5;
//        create_prm.request.occupancy_threshold = 20;
//        create_prm.request.total_attempts = 1000;
//        create_prm.request.nr_waypoints = 10;
//        create_prm.request.min_distance = 0.3;
//        create_prm.request.casting_distance = 2.0;
//        create_prm.request.connecting_distance = 8.0;
//        create_prm.request.occupancy_threshold = 10;
//        create_prm.request.total_attempts = 1000;
        create_prm.request.nr_waypoints = 1;
        create_prm.request.min_distance = 0.3;
        create_prm.request.casting_distance = 2.0;
        create_prm.request.connecting_distance = 8.0;
        create_prm.request.occupancy_threshold = 10;
        create_prm.request.total_attempts = 1000;
        if (!roadmap_service.call(create_prm)) {
            ROS_ERROR("KCL: (LookGrasp) Failed to call the road map service.");
            return;
        }
        ROS_INFO("KCL: (LookGrasp) Road map service returned.");

        // get object point
        //ROS_INFO("KCL: (LookGrasp) Waiting for explore point.");
        //std::string obWPID("explore_waypoint");
        //makePoint(obWPID);

        // get goal point
        //ROS_INFO("KCL: (LookGrasp) Waiting for goal location point.");
        //std::string goWPID("goal_waypoint");
        //makePoint(goWPID);

//        // Setup the goto waypoint goal.
//        ROS_INFO("KCL: (LookGrasp) Setting up goto waypoint goal.");
//        rosplan_knowledge_msgs::KnowledgeItem waypoint_goal;
//        waypoint_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//        waypoint_goal.attribute_name = "visited";
//        diagnostic_msgs::KeyValue kv1;
//        kv1.key = "wp"; kv1.value = "wp0";
//        waypoint_goal.values.push_back(kv1);

//        // Add the goto waypoint goal.
//        ROS_INFO("KCL: (LookGrasp) Adding goto waypoint goal.");
        rosplan_knowledge_msgs::KnowledgeUpdateService add_goal;
//        add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
//        add_goal.request.knowledge = waypoint_goal;
//        if (!knowledge_update_client.call(add_goal)) {
//            ROS_ERROR("KCL: (LookGrasp) Could not add goto waypoint goal to the knowledge base.");
//            exit(-1);
//        }

//        // Setup the explore area goal.
//        ROS_INFO("KCL: (LookGrasp) Setting up explore area goal.");
//        rosplan_knowledge_msgs::KnowledgeItem explore_goal;
//        explore_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//        explore_goal.attribute_name = "explored";
//        diagnostic_msgs::KeyValue kv2;
//        kv2.key = "wp"; kv2.value = "wp0";
//        explore_goal.values.push_back(kv2);

//        // Add the explore area goal.
//        ROS_INFO("KCL: (LookGrasp) Adding explore area goal.");
//        add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
//        add_goal.request.knowledge = explore_goal;
//        if (!knowledge_update_client.call(add_goal)) {
//            ROS_ERROR("KCL: (LookGrasp) Could not add explore goal to the knowledge base.");
//            exit(-1);
//        }

//        // Run the planner.
//        // This generates the plan (with goto and explore goals).
//        ROS_INFO("KCL: (LookGrasp) Running planner.");
        std_srvs::Empty dummy;
//        if (!run_planner_client.call(dummy)) {
//            ROS_ERROR("KCL: (LookGrasp) Failed to run the planning system.");
//            exit(-1);
//        }
//        ROS_INFO("KCL: (LookGrasp) Planning system returned.");

        // get all objects
        rosplan_knowledge_msgs::GetInstanceService getInstances;
        getInstances.request.type_name = "object";
        if (!get_instance_client.call(getInstances)) {
            ROS_ERROR("KCL: (LookGrasp) Failed to get all the object instances.");
            return;
        }
        ROS_INFO("KCL: (LookGrasp) Received all the object instances.");
        ROS_INFO("KCL: (LookGrasp) There are %lu instances.", getInstances.response.instances.size());

        // Setup the pickup object goal.
        ROS_INFO("KCL: (LookGrasp) Setting up pickup object goal.");
        rosplan_knowledge_msgs::KnowledgeItem pickup_goal;
        pickup_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        pickup_goal.attribute_name = "grasped";
        diagnostic_msgs::KeyValue kv;
        kv.key = "wp"; kv.value = "wp0";  // need to determine wp0 (i.e. the pickup location)!!
        pickup_goal.values.push_back(kv);
        kv.key = "o"; kv.value = "object1";
        pickup_goal.values.push_back(kv);

        // Add the pickup object goal.
        ROS_INFO("KCL: (LookGrasp) Adding pickup goal.");
        add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
        add_goal.request.knowledge = pickup_goal;
        if (!knowledge_update_client.call(add_goal)) {
            ROS_ERROR("KCL: (LookGrasp) Could not add pickup object goal to the knowledge base.");
            exit(-1);
        }

        // This will generate a new plan
        ROS_INFO("KCL: (LookGrasp) Running planner to grasp object.");
        // Run the planner agian.
        if (!run_planner_client.call(dummy)) {
            ROS_ERROR("KCL: (LookGrasp) Failed to run the planning system.");
            exit(-1);
        }
        ROS_INFO("KCL: (LookGrasp) Planning system returned.");

//        int ob_count = 0;
//        for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci) {
//            ++ob_count;

//            // add PREDICATE tidy_location
//            rosplan_knowledge_msgs::KnowledgeUpdateService tlSrv;
//            tlSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
//            tlSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//            tlSrv.request.knowledge.attribute_name = "tidy_location";
//            diagnostic_msgs::KeyValue object;
//            object.key = "o";
//            object.value = (*ci);
//            tlSrv.request.knowledge.values.push_back(object);
//            diagnostic_msgs::KeyValue location;
//            location.key = "wp";
//            location.value = "goal";
//            tlSrv.request.knowledge.values.push_back(location);
//            if (!knowledge_update_client.call(tlSrv))
//                ROS_ERROR("KCL: (LookGrasp) error adding knowledge");

//            // Remove tidy_location_unknown for this object.
//            rosplan_knowledge_msgs::KnowledgeUpdateService tidyLocationUnknownSrv;
//            tidyLocationUnknownSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
//            tidyLocationUnknownSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//            tidyLocationUnknownSrv.request.knowledge.attribute_name = "tidy_location_unknown";
//            object.key = "o";
//            object.value = (*ci);
//            tidyLocationUnknownSrv.request.knowledge.values.push_back(object);
//            if (!knowledge_update_client.call(tidyLocationUnknownSrv))
//                ROS_ERROR("KCL: (LookGrasp) error removing tidy_location_unknown predicate");

//            ROS_INFO("KCL: (LookGrasp) fetching object %s", (*ci).c_str());
////            // fetch position of object from message store
////            std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
////            if(message_store.queryNamed<geometry_msgs::PoseStamped>(*ci, results)) {
////                if(results.size()<1) {
////                    ROS_ERROR("KCL: (LookGrasp) aborting pushing location; no matching obID %s", (*ci).c_str());
////                    return;
////                }
////                if(results.size()>1)
////                    ROS_ERROR("KCL: (LookGrasp) multiple waypoints share the same wpID");
////            } else {
////                ROS_ERROR("KCL: (LookGrasp) could not query message store to fetch object pose");
////                return;
////            }

////            // calculate pushing pose for object and new point
////            geometry_msgs::PoseStamped &objPose = *results[0];
////            float d = sqrt(
////                (objPose.pose.position.x - goal_pose.pose.position.x)*(objPose.pose.position.x - goal_pose.pose.position.x) +
////                (objPose.pose.position.y - goal_pose.pose.position.y)*(objPose.pose.position.y - goal_pose.pose.position.y));
////            geometry_msgs::PoseStamped pushingPose;
////            pushingPose.header.frame_id = "/map";
////            float startDistance = 0.30;
////            pushingPose.pose.position.x = objPose.pose.position.x + startDistance*(objPose.pose.position.x - goal_pose.pose.position.x)/d;
////            pushingPose.pose.position.y = objPose.pose.position.y + startDistance*(objPose.pose.position.y - goal_pose.pose.position.y)/d;

////            float angle = atan2(goal_pose.pose.position.y - objPose.pose.position.y, goal_pose.pose.position.x - objPose.pose.position.x);
////            if(isnan(angle)) angle = 0;

////            pushingPose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

//            geometry_msgs::PoseStamped pushingPose;
//            pushingPose.header.frame_id = "/map";
//            pushingPose.pose.position.x = 1.0;
//            pushingPose.pose.position.y = 1.0;
//            float angle = 0;
//            pushingPose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

//            // add pushing location for this object
//            rosplan_knowledge_msgs::AddWaypoint addWPSrv;
//            std::stringstream ss_pp;
//            ss_pp << "push_start_location_" << *ci;
//            addWPSrv.request.id = ss_pp.str();
//            addWPSrv.request.waypoint = pushingPose;
//            addWPSrv.request.connecting_distance = 5;
//            addWPSrv.request.occupancy_threshold = 20;
//            if (!add_waypoint_client.call(addWPSrv)) {
//                ROS_ERROR("KCL: (LookGrasp) Failed to add a new waypoint for the object");
//            }
//            ROS_INFO("KCL: (LookGrasp) Road map service returned");

//            // add PREDICATE push_location
//            rosplan_knowledge_msgs::KnowledgeUpdateService ppSrv;
//            ppSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
//            ppSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//            ppSrv.request.knowledge.attribute_name = "push_location";
//            ppSrv.request.knowledge.values.push_back(object);
//            location.value = ss_pp.str();
//            ppSrv.request.knowledge.values.push_back(location);
//            if (!knowledge_update_client.call(ppSrv))
//                ROS_ERROR("KCL: (LookGrasp) error adding knowledge");
//        }

//        // Run the planner agian.
//        if (!run_planner_client.call(dummy)) {
//            ROS_ERROR("KCL: (LookGrasp) Failed to run the planning system.");
//            exit(-1);
//        }
//        ROS_INFO("KCL: (LookGrasp) Planning system returned.");


    } // end of function runDemo()

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_look_grasp_server");
    ros::NodeHandle nh;

    // Adjust camera here (HACK but good enough for now)
//    ROS_INFO("Tilting camera ...");
//    system("rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- 0.8");
//    sleep(5);
//    system("rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- 0.8");
//    sleep(5);

    ROS_INFO("--- STARTING DEMO ---");

    // create PDDL action subscriber
    Look_Grasp::LookGraspExecutor demo_node(nh);

    // listen for pointing
    //ros::Subscriber pointing_pose_sub = nh.subscribe("/squirrel_person_tracker/pointing_pose", 1, &KCL_rosplan::SimpleDemoExecutor::receivePointLocation, &sde);

    // listen for action dispatch
    ROS_INFO("KCL: (LookGrasp) Ready to receive");

    demo_node.runDemo();
    return 0;
}



