#ifndef __C_KEY_VALUE_PAIR_H__
#define __C_KEY_VALUE_PAIR_H__


#include <list>
#include <string>

#include <designator_integration_msgs/KeyValuePair.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


enum ValueType {
  STRING = 0,
  FLOAT = 1,
  DATA = 2,
  LIST = 3,
  POSESTAMPED = 4
};


class CKeyValuePair {
 private:
  enum ValueType m_evtType;
  int m_nID;
  int m_nParent;
  string m_strKey;
  string m_strValue;
  double m_fValue;
  char *m_acData;
  geometry_msgs::PoseStamped m_psPoseStampedValue;

 protected:
  list<CKeyValuePair*> m_lstChildren;

 public:
  CKeyValuePair();
  CKeyValuePair(string strKey);
  CKeyValuePair(string strKey, string strValue);
  CKeyValuePair(string strKey, float fValue);
  CKeyValuePair(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  CKeyValuePair(designator_integration_msgs::KeyValuePair kvpContent);
  ~CKeyValuePair();

  void init();
  void fillWithKeyValueContent(designator_integration_msgs::KeyValuePair kvpContent);
  enum ValueType type();
  
  string stringValue();
  string stringValue(string strChildKey);
  float floatValue();
  float floatValue(string strChildKey);
  
  int id();
  int parent();
  string key();
  geometry_msgs::PoseStamped poseStampedValue();
  
  void setID(int nID);
  void setParent(int nParent);
  
  void setStringValue(string strValue);
  void setFloatValue(float fValue);
  void setPoseStampedValue(geometry_msgs::PoseStamped psPoseStampedValue);
  void setValue(string strValue);
  void setValue(float fValue);
  void setValue(geometry_msgs::PoseStamped psPoseStampedValue);
  
  void setStringValue(string strKey, string strValue);
  void setFloatValue(string strKey, float fValue);
  void setPoseStampedValue(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  void setValue(string strKey, string strValue);
  void setValue(string strKey, float fValue);
  void setValue(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  
  void setKey(string strKey);
  void setType(enum ValueType evtType);
  
  void addChild(CKeyValuePair *kvpAdd);
  list<CKeyValuePair*> children();
  
  void printPair(int nSpaceOffset, bool bOffsetRegular = true, bool bNewline = true);
  void printSpaces(int nSpaces);
  
  CKeyValuePair *addStringChild(string strKey, string strValue);
  CKeyValuePair *addFloatChild(string strKey, float fValue);
  CKeyValuePair *addPoseStampedChild(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  
  CKeyValuePair *addChild(string strKey);
  CKeyValuePair *addChild(string strKey, string strValue);
  CKeyValuePair *addChild(string strKey, float fValue);
  CKeyValuePair *addChild(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  
  vector<designator_integration_msgs::KeyValuePair> serializeToMessage(int nParent, int nID);
  CKeyValuePair *childForKey(string strKey);
};


#endif /* __C_KEY_VALUE_PAIR_H__ */
