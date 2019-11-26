#ifndef __DESIGNATOR_H__
#define __DESIGNATOR_H__


// System
#include <list>
#include <string>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <memory>

// ROS
#include <designator_integration_msgs/Designator.h>

// Private
#include <designators/KeyValuePair.h>
#include <designators/config.h>


namespace designator_integration {
  class Designator : public KeyValuePair {
  public:
    typedef std::shared_ptr<Designator> Ptr;
    
    typedef enum DesignatorType_ {
      OBJECT = 0,
      ACTION = 1,
      LOCATION = 2,
      HUMAN = 3,
      UNKNOWN = 4
    } DesignatorType;

  private:
    DesignatorType m_edtType;

  public:
    Designator();
    Designator(Designator* desigTemplate);
    Designator(designator_integration_msgs::Designator desigContent);
    Designator(DesignatorType edtType, KeyValuePair* kvpDescription = NULL);
    Designator(DesignatorType edtType, std::list<KeyValuePair*> lstDescription);

    void fillFromDescription(DesignatorType edtType, std::list<KeyValuePair*> lstDescription);
    void setDescription(std::list<KeyValuePair*> lstDescription);
    std::list<KeyValuePair*> description();

    void setType(DesignatorType edtType);

    void fillFromDesignatorMsg(designator_integration_msgs::Designator desigContent);
    DesignatorType type();
    void printDesignator();
    
    designator_integration_msgs::Designator serializeToMessage(bool setParent = true);
    
    // The following methods are only available if compiled with rapidJSON
#ifdef DESIGNATOR_WITH_JSON
    void fillFromJSON(const std::string &json);
    std::string serializeToJSON();
    static std::string serializeToJSON(std::vector<Designator> &designators);
#endif // DESIGNATOR_WITH_JSON
    
    template<class ... Args>
      static Designator::Ptr create(Args ... args) {
      return std::make_shared<Designator>(std::forward<Args>(args)...);
    }
    
    static std::string typeToString(DesignatorType dtType);
  };
}


#endif /* __DESIGNATOR_H__ */
