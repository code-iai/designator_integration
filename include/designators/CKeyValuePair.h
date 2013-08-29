#ifndef __C_KEY_VALUE_PAIR_H__
#define __C_KEY_VALUE_PAIR_H__


#include <list>
#include <string>

#include <designator_integration_msgs/KeyValuePair.h>
#include <ros/ros.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


enum ValueType {
  STRING = designator_integration_msgs::KeyValuePair::TYPE_STRING,
  FLOAT = designator_integration_msgs::KeyValuePair::TYPE_FLOAT,
  DATA = designator_integration_msgs::KeyValuePair::TYPE_DATA,
  LIST = designator_integration_msgs::KeyValuePair::TYPE_LIST,
  POSESTAMPED = designator_integration_msgs::KeyValuePair::TYPE_POSESTAMPED,
  POSE = designator_integration_msgs::KeyValuePair::TYPE_POSE,
  DESIGNATOR_ACTION = designator_integration_msgs::KeyValuePair::TYPE_DESIGNATOR_ACTION,
  DESIGNATOR_OBJECT = designator_integration_msgs::KeyValuePair::TYPE_DESIGNATOR_OBJECT,
  DESIGNATOR_LOCATION = designator_integration_msgs::KeyValuePair::TYPE_DESIGNATOR_LOCATION
};


class CKeyValuePair {
 private:
  enum ValueType m_evtType;
  int m_nID;
  int m_nParent;
  string m_strKey;
  string m_strValue;
  double m_fValue;
  char *m_acValue;
  unsigned int m_unValueLength;
  geometry_msgs::Pose m_posPoseValue;
  geometry_msgs::PoseStamped m_psPoseStampedValue;

 protected:
  list<CKeyValuePair*> m_lstChildren;

 public:
  CKeyValuePair();
  CKeyValuePair(string strKey);
  CKeyValuePair(string strKey, string strValue);
  CKeyValuePair(string strKey, float fValue);
  CKeyValuePair(string strKey, char *acValue, unsigned int unLength);
  CKeyValuePair(string strKey, geometry_msgs::Pose posPoseValue);
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
  geometry_msgs::PoseStamped poseStampedValue();
  geometry_msgs::PoseStamped poseStampedValue(string strChildKey);
  geometry_msgs::Pose poseValue();
  geometry_msgs::Pose poseValue(string strChildKey);
  char *dataValue();
  unsigned int dataValueLength();
  char *dataValue(unsigned int &unLength);
  
  int id();
  int parent();
  string key();
  
  void setID(int nID);
  void setParent(int nParent);
  
  void setValue(string strValue);
  void setValue(float fValue);
  void setValue(geometry_msgs::PoseStamped psPoseStampedValue);
  void setValue(geometry_msgs::Pose psPoseValue);
  void setValue(char *acValue, unsigned int unLength);
  void setValue(enum ValueType evtType, list<CKeyValuePair*> lstDescription);
  void clearDataValue();
  
  void setValue(string strKey, string strValue);
  void setValue(string strKey, float fValue);
  void setValue(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  void setValue(string strKey, geometry_msgs::Pose psPoseValue);
  void setValue(string strKey, char *acValue, int nLength);
  
  // Adding designator descriptions as values
  void setValue(string strKey, enum ValueType evtType, list<CKeyValuePair*> lstDescription);
  void setLocationDesignatorDescription(string strKey, enum ValueType evtType, list<CKeyValuePair*> lstDescription);
  void setActionDesignatorDescription(string strKey, list<CKeyValuePair*> lstDescription);
  void setObjectDesignatorDescription(string strKey, list<CKeyValuePair*> lstDescription);
  
  void setKey(string strKey);
  void setType(enum ValueType evtType);
  
  CKeyValuePair *addChild(string strKey);
  list<CKeyValuePair*> children();
  
  void printPair(int nSpaceOffset, bool bOffsetRegular = true, bool bNewline = true);
  void printSpaces(int nSpaces);
  
  void addChild(CKeyValuePair *ckvpChildAdd);
  CKeyValuePair *addChild(string strKey, string strValue);
  CKeyValuePair *addChild(string strKey, float fValue);
  CKeyValuePair *addChild(string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
  CKeyValuePair *addChild(string strKey, geometry_msgs::Pose psPoseValue);
  CKeyValuePair *addChild(string strKey, char *acValue, unsigned int unLength);
  CKeyValuePair *addChild(string strKey, enum ValueType evtType, list<CKeyValuePair*> lstDescription);
  
  vector<designator_integration_msgs::KeyValuePair> serializeToMessage(int nParent, int nID);
  CKeyValuePair *childForKey(string strKey);
  
  void clear();
  
  CKeyValuePair* copy();
};


#endif /* __C_KEY_VALUE_PAIR_H__ */
