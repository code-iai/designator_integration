#include <string>
#include <iostream>
#include <list>

#include <ros/ros.h>
#include <designator_integration_msgs/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

#include <designators/CDesignator.h>

using namespace std;


bool desigCommCB(designator_integration_msgs::DesignatorCommunication::Request &req,
		 designator_integration_msgs::DesignatorCommunication::Response &res) {
  CDesignator *desigRequest = new CDesignator(req.request);
  CDesignator *desigResponse = new CDesignator();
  desigResponse->setType(OBJECT);
  desigResponse->addValue("name", desigRequest->getStringValue("name"));
  desigResponse->addValue("height", 0.2);
  
  geometry_msgs::PoseStamped psTest;
  psTest.pose.position.x = 0.2;
  
  desigResponse->addValue("pose", psTest);
  
  res.response.push_back(desigResponse->serializeToMessage());
  
  return true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "desig_int_test");
  ros::NodeHandle nhPrivate("~");
  ros::ServiceServer srvDesigComm = nhPrivate.advertiseService("/designator_comm",
							       desigCommCB);
  
  ros::spin();
  
  return 0;
}
