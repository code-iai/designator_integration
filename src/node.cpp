#include <string>
#include <iostream>
#include <list>

#include <ros/ros.h>
#include <designator_integration_msgs/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

#include <designators/CDesignator.h>

using namespace std;


ros::Publisher pub;


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
  ros::NodeHandle nh;
  ros::ServiceServer srvDesigComm = nhPrivate.advertiseService("/designator_comm",
							       desigCommCB);
  
  string strTopic = "/uima/trigger";
  pub = nh.advertise<designator_integration_msgs::Designator>(strTopic, 1);
  
  CDesignator *desigTest = new CDesignator();
  desigTest->setType(ACTION);
  
  desigTest->setValue("shape", "box");
  desigTest->setValue("color", "red");
  
  bool bSent = false;
  
  while(ros::ok()) {
    ros::spinOnce();
    
    if(!bSent) {
      pub.publish(desigTest->serializeToMessage());
      
      cout << "Sent this designator to topic '" << strTopic << "':" << endl;
      desigTest->printDesignator();
      bSent = true;
    }
    
    ros::Duration(0.1).sleep();
  }
  
  delete desigTest;
  
  return 0;
}
