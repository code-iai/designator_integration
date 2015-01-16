// System
#include <string>
#include <iostream>
#include <list>

// ROS
#include <ros/ros.h>
#include <designator_integration_msgs/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Designator Integration
#include <designators/Designator.h>


ros::Publisher pub;


bool desigCommCB(designator_integration_msgs::DesignatorCommunication::Request &req,
		 designator_integration_msgs::DesignatorCommunication::Response &res) {
  designator_integration::Designator* desigRequest = new designator_integration::Designator(req.request.designator);
  designator_integration::Designator* desigResponse = new designator_integration::Designator();
  
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
  ros::NodeHandle nh;
  ros::ServiceServer srvDesigComm = nhPrivate.advertiseService("/designator_comm",
							       desigCommCB);
  
  std::string strTopic = "/uima/trigger";
  pub = nh.advertise<designator_integration_msgs::Designator>(strTopic, 1);
  
  designator_integration::Designator *desigTest = new designator_integration::Designator();
  desigTest->setType(ACTION);
  
  desigTest->setValue("shape", "box");
  desigTest->setValue("color", "red");
  
  designator_integration::KeyValuePair* ckvpTest1 = desigTest->addChild("GRASP-POSE", true);
  ckvpTest1->setValue("test-content 1");

  designator_integration::KeyValuePair* ckvpTest2 = desigTest->addChild("GRASP-POSE", true);
  ckvpTest2->setValue("test-content 2");
  
  bool bSent = false;
  
  while(ros::ok()) {
    ros::spinOnce();
    
    if(!bSent) {
      pub.publish(desigTest->serializeToMessage());
      
      std::cout << "Sent this designator to topic '" << strTopic << "':" << std::endl;
      desigTest->printDesignator();
      bSent = true;
    }
    
    ros::Duration(0.1).sleep();
  }
  
  delete desigTest;
  
  return 0;
}
