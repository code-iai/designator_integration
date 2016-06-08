#include <designators/Designator.h>


namespace designator_integration {
  Designator::Designator() : KeyValuePair() {
    this->m_edtType = OBJECT;
  }

  Designator::Designator(Designator* cdTemplate) {
    this->setType(cdTemplate->type());
    this->setDescription(cdTemplate->description());
  }

  Designator::Designator(designator_integration_msgs::Designator desigContent) {
    this->fillFromDesignatorMsg(desigContent);
  }

  Designator::Designator(DesignatorType edtType, KeyValuePair* ckvpDescription) {
    if(ckvpDescription) {
      this->fillFromDescription(edtType, ckvpDescription->children());
    } else {
      this->setType(edtType);
    }
  }

  Designator::Designator(DesignatorType edtType, std::list<KeyValuePair*> lstDescription) {
    this->fillFromDescription(edtType, lstDescription);
  }
  
  void Designator::fillFromDescription(DesignatorType edtType, std::list<KeyValuePair*> lstDescription) {
    std::list<KeyValuePair*> lstDescriptionNew;
  
    for(KeyValuePair* kvpDesc : lstDescription) {
      lstDescriptionNew.push_back(kvpDesc->copy());
    }
    
    this->setDescription(lstDescriptionNew);
    this->setType(edtType);
  }
  
  void Designator::setDescription(std::list<KeyValuePair*> lstDescription) {
    m_lstChildren = lstDescription;
  }
  
  std::list<KeyValuePair*> Designator::description() {
    return m_lstChildren;
  }
  
  void Designator::setType(DesignatorType edtType) {
    m_edtType = edtType;
    
    ValueType evtType = DESIGNATOR_OBJECT;
    switch(edtType) {
    case ACTION:
      evtType = DESIGNATOR_ACTION;
      break;
    
    case OBJECT:
      evtType = DESIGNATOR_OBJECT;
      break;
    
    case LOCATION:
      evtType = DESIGNATOR_LOCATION;
      break;

    case HUMAN:
      evtType = DESIGNATOR_HUMAN;
      break;
      
    default:
      break;
    }
    
    m_evtType = evtType;
  }
  
  void Designator::fillFromDesignatorMsg(designator_integration_msgs::Designator desigContent) {
    m_edtType = (DesignatorType)desigContent.type;
  
    std::list<KeyValuePair*> lstKVPs;
    for(designator_integration_msgs::KeyValuePair kvpmsgPair : desigContent.description) {
      KeyValuePair* ckvpCurrent = new KeyValuePair(kvpmsgPair);
      lstKVPs.push_back(ckvpCurrent);
    }
    
    while(lstKVPs.size() > 0) {
      for(KeyValuePair* kvpCurrent : lstKVPs) {
        bool bFoundChild = false;

        for(KeyValuePair* kvpInspect : lstKVPs) {
          if(kvpInspect != kvpCurrent && kvpInspect->parent() == kvpCurrent->id()) {
            bFoundChild = true;
            break;
          }
        }

        if(!bFoundChild) {
          bool bFoundParent = false;

          for(KeyValuePair* kvpInspect : lstKVPs) {
            if(kvpInspect != kvpCurrent && kvpInspect->id() == kvpCurrent->parent()) {
              kvpInspect->addChild(kvpCurrent);
              lstKVPs.remove(kvpCurrent);
              bFoundParent = true;
              break;
            }
          }

          if(bFoundParent) {
            break;
          } else {
            m_lstChildren.push_back(kvpCurrent);
            lstKVPs.remove(kvpCurrent);
            break;
          }
        }
      }
    }
  }
  
  Designator::DesignatorType Designator::type() {
    return m_edtType;
  }
  
  void Designator::printDesignator() {
    for(KeyValuePair* kvpCurrent : m_lstChildren) {
      kvpCurrent->printPair(0);
      std::cout << std::endl;
    }
  }
  
  designator_integration_msgs::Designator Designator::serializeToMessage(bool setParent) {
    designator_integration_msgs::Designator msgDesig;
    msgDesig.type = (int)m_edtType;
    
    int nHighestID = 0;
    std::vector<designator_integration_msgs::KeyValuePair> vecKVPs;
    
    for(KeyValuePair* kvpCurrent : m_lstChildren) {
      std::vector<designator_integration_msgs::KeyValuePair> vecKVPMsgs = kvpCurrent->serializeToMessage(0, nHighestID + 1,setParent);
      
      for(designator_integration_msgs::KeyValuePair kvpmsgCurrent : vecKVPMsgs) {
        if(kvpmsgCurrent.id > nHighestID) {
          nHighestID = kvpmsgCurrent.id;
        }

        vecKVPs.push_back(kvpmsgCurrent);
      }
    }
    
    msgDesig.description = vecKVPs;
    
    return msgDesig;
  }
}
