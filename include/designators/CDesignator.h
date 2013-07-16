#ifndef __C_DESIGNATOR_H__
#define __C_DESIGNATOR_H__


// System
#include <list>
#include <string>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <cstring>
#include <iostream>

// ROS
#include <designator_integration_msgs/Designator.h>

// Private
#include <designators/CKeyValuePair.h>

using namespace std;


enum DesignatorType {
  OBJECT = 0,
  ACTION = 1,
  LOCATION = 2,
  UNKNOWN = 3
};


class CDesignator : public CKeyValuePair {
 private:
  enum DesignatorType m_edtType;

 public:
  CDesignator();
  CDesignator(designator_integration_msgs::Designator desigContent);
  CDesignator(enum DesignatorType edtType, CKeyValuePair* ckvpDescription);
  CDesignator(enum DesignatorType edtType, list<CKeyValuePair*> lstDescription);
  
  void fillFromDescription(enum DesignatorType edtType, list<CKeyValuePair*> lstDescription);
  void setDescription(list<CKeyValuePair*> lstDescription);
  list<CKeyValuePair*> description();
  
  void setType(enum DesignatorType edtType);
  
  void fillFromDesignatorMsg(designator_integration_msgs::Designator desigContent);
  enum DesignatorType type();
  void printDesignator();
  
  designator_integration_msgs::Designator serializeToMessage();
};


#endif /* __C_DESIGNATOR_H__ */
