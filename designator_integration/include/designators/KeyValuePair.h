#ifndef __KEY_VALUE_PAIR_H__
#define __KEY_VALUE_PAIR_H__


// System
#include <list>
#include <string>

// ROS
#include <ros/ros.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

// Private
#include <designator_integration_msgs/KeyValuePair.h>


namespace designator_integration {
  class KeyValuePair {
  public:
    typedef enum ValueType_ {
      STRING = designator_integration_msgs::KeyValuePair::TYPE_STRING,
      FLOAT = designator_integration_msgs::KeyValuePair::TYPE_FLOAT,
      DATA = designator_integration_msgs::KeyValuePair::TYPE_DATA,
      LIST = designator_integration_msgs::KeyValuePair::TYPE_LIST,
      POSESTAMPED = designator_integration_msgs::KeyValuePair::TYPE_POSESTAMPED,
      POSE = designator_integration_msgs::KeyValuePair::TYPE_POSE,
      DESIGNATOR_ACTION = designator_integration_msgs::KeyValuePair::TYPE_DESIGNATOR_ACTION,
      DESIGNATOR_OBJECT = designator_integration_msgs::KeyValuePair::TYPE_DESIGNATOR_OBJECT,
      DESIGNATOR_LOCATION = designator_integration_msgs::KeyValuePair::TYPE_DESIGNATOR_LOCATION
    } ValueType;
    
  private:
    int m_nID;
    int m_nParent;
    std::string m_strKey;
    std::string m_strValue;
    double m_dValue;
    char* m_acValue;
    unsigned int m_unValueLength;
    geometry_msgs::Pose m_posPoseValue;
    geometry_msgs::PoseStamped m_psPoseStampedValue;
    bool m_bIsAtom;
    
  protected:
    ValueType m_evtType;
    std::list<KeyValuePair*> m_lstChildren;
    
  public:
    KeyValuePair();
    KeyValuePair(std::string strKey);
    KeyValuePair(std::string strKey, std::string strValue);
    KeyValuePair(std::string strKey, double dValue);
    KeyValuePair(std::string strKey, char* acValue, unsigned int unLength);
    KeyValuePair(std::string strKey, geometry_msgs::Pose posPoseValue);
    KeyValuePair(std::string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
    KeyValuePair(designator_integration_msgs::KeyValuePair kvpContent);
    KeyValuePair(std::list<KeyValuePair*> lstChildren);
    ~KeyValuePair();
    
    void init();
    void fillWithKeyValueContent(designator_integration_msgs::KeyValuePair kvpContent);
    ValueType type();
    
    bool isAtom();
    void setIsAtom(bool bIsAtom);
    
    std::string stringValue();
    std::string stringValue(std::string strChildKey);
    double floatValue();
    double floatValue(std::string strChildKey);
    geometry_msgs::PoseStamped poseStampedValue();
    geometry_msgs::PoseStamped poseStampedValue(std::string strChildKey);
    geometry_msgs::Pose poseValue();
    geometry_msgs::Pose poseValue(std::string strChildKey);
    char* dataValue();
    unsigned int dataValueLength();
    char* dataValue(unsigned int& unLength);
    
    int id();
    int parent();
    std::string key();
    
    void setID(int nID);
    void setParent(int nParent);
    
    void addAtom(std::string strValue);
    void addAtom(double dValue);
    void addAtom(geometry_msgs::PoseStamped psPoseStampedValue);
    void addAtom(geometry_msgs::Pose psPoseValue);
    
    void setValue(std::string strValue);
    void setValue(double dValue);
    void setValue(geometry_msgs::PoseStamped psPoseStampedValue);
    void setValue(geometry_msgs::Pose psPoseValue);
    void setValue(void* vdValue, unsigned int unLength);
    void setValue(ValueType evtType, std::list<KeyValuePair*> lstDescription);
    void clearDataValue();
    
    void setValue(std::string strKey, std::string strValue);
    void setValue(std::string strKey, double dValue);
    void setValue(std::string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
    void setValue(std::string strKey, geometry_msgs::Pose psPoseValue);
    void setValue(std::string strKey, char *acValue, int nLength);
    
    // Adding designator descriptions as values
    void setValue(std::string strKey, ValueType evtType, std::list<KeyValuePair*> lstDescription);
    void setLocationDesignatorDescription(std::string strKey, ValueType evtType, std::list<KeyValuePair*> lstDescription);
    void setActionDesignatorDescription(std::string strKey, std::list<KeyValuePair*> lstDescription);
    void setObjectDesignatorDescription(std::string strKey, std::list<KeyValuePair*> lstDescription);
    
    void setKey(std::string strKey);
    void setType(ValueType evtType);
    
    KeyValuePair* addChild(std::string strKey, bool bAppendNew = false);
    std::list<KeyValuePair*> children();
    void setChildren(std::list<KeyValuePair*> lstChildren);
    
    void printPair(int nSpaceOffset, bool bOffsetRegular = true, bool bNewline = true);
    void printSpaces(int nSpaces);
    
    void addChild(KeyValuePair* ckvpChildAdd);
    KeyValuePair* addChild(std::string strKey, std::string strValue);
    KeyValuePair* addChild(std::string strKey, double dValue);
    KeyValuePair* addChild(std::string strKey, geometry_msgs::PoseStamped psPoseStampedValue);
    KeyValuePair* addChild(std::string strKey, geometry_msgs::Pose psPoseValue);
    KeyValuePair* addChild(std::string strKey, char* acValue, unsigned int unLength);
    KeyValuePair* addChild(std::string strKey, ValueType evtType, std::list<KeyValuePair*> lstDescription);
    
    std::vector<designator_integration_msgs::KeyValuePair> serializeToMessage(int nParent, int nID);
    KeyValuePair* childForKey(std::string strKey);
    bool removeChildForKey(std::string strKey);
    
    void clear();
    
    KeyValuePair* copy();
    
    std::list<std::string> keys();
  };
}

#endif /* __KEY_VALUE_PAIR_H__ */
