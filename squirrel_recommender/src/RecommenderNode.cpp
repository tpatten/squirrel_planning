#include <ros/ros.h>
#include <squirrel_prediction_msgs/RecommendRelations.h>

 bool uibk_recommend(squirrel_prediction_msgs::RecommendRelations::Request  &req,
          squirrel_prediction_msgs::RecommendRelations::Response &res){
   //res.sum = req.a + req.b;
   //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
   //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
   return true;
 }


int main( int argc, char *argv[] ){
  ros::init(argc, argv, "recommender");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("recommender", uibk_recommend);
  ROS_INFO("(squirrel recommender) Ready for predicting missing values.");
  ros::spin();

  return 0;
}

