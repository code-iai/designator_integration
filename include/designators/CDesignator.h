#ifndef __C_DESIGNATOR_H__
#define __C_DESIGNATOR_H__


#include <list>
#include <string>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <cstring>
#include <iostream>

#include <designator_integration_msgs/Designator.h>

#include <designators/CKeyValuePair.h>

using namespace std;


enum DesignatorType {
  OBJECT = 0,
  ACTION = 1,
  LOCATION = 2,
  UNKNOWN = 3
};


class CDesignator {
 private:
  enum DesignatorType m_edtType;
  list<CKeyValuePair*> m_lstKeyValuePairs;

 public:
  CDesignator();
  CDesignator(designator_integration_msgs::Designator desigContent);
  
  void setType(enum DesignatorType edtType);
  
  void fillFromDesignatorMsg(designator_integration_msgs::Designator desigContent);
  enum DesignatorType type();
  void printDesignator();
  
  CKeyValuePair* addValue(string strKey);
  CKeyValuePair* addValue(string strKey, string strValue);
  CKeyValuePair* addValue(string strKey, float fValue);
  CKeyValuePair* addValue(string strKey, geometry_msgs::PoseStamped psValue);
  
  CKeyValuePair* keyValuePairForKey(string strKey);
  
  designator_integration_msgs::Designator serializeToMessage();
  vector<designator_integration_msgs::KeyValuePair> serializeKeyValuePair(CKeyValuePair *ckvpSerialize, int nParent, int nHighestID);
  
  CKeyValuePair* getPairForKey(string strKey);
  
  string getStringValue(string strKey);
  float getFloatValue(string strKey);
  geometry_msgs::PoseStamped getPoseStampedValue(string strKey);
};


#endif /* __C_DESIGNATOR_H__ */
