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
  CDesignator *desigRequest = new CDesignator(req.request.designator);
  CDesignator *desigResponse = new CDesignator();
  desigResponse->setType(OBJECT);
  desigResponse->setValue("name", desigRequest->stringValue("name"));
  desigResponse->setValue("height", 0.2);
  
  geometry_msgs::PoseStamped psTest;
  psTest.pose.position.x = 0.2;
  
  desigResponse->setValue("pose", psTest);
  
  res.response.designators.push_back(desigResponse->serializeToMessage());
  
  return true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "desig_int_test");
  ros::NodeHandle nhPrivate("~");
  ros::ServiceServer srvDesigComm = nhPrivate.advertiseService("/designator_comm",
							       desigCommCB);
  
  CDesignator *desigTest = new CDesignator();
  
  desigTest->setValue("test", "value");
  desigTest->setValue("test2", 0.5);
  CKeyValuePair *c1 = desigTest->addChild("subnode");
  c1->setValue("val1", 0.3);
  c1->setValue("val2", "somevalue");
  CKeyValuePair *c2 = c1->addChild("subnode-second-level");
  c2->setValue("val3", "wow");
  c2->setValue("val4", "wow2");
  c1->setValue("val5", "somevalue2");
  desigTest->setValue("test-value", 1.9);
  desigTest->printDesignator();
  
  delete desigTest;
  
  ros::spin();
  
  return 0;
}
