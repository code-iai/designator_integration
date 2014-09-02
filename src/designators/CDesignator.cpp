#include <designators/CDesignator.h>


CDesignator::CDesignator() : CKeyValuePair() {
  this->m_edtType = OBJECT;
}

CDesignator::CDesignator(CDesignator* cdTemplate) {
  this->setType(cdTemplate->type());
  this->setDescription(cdTemplate->description());
}

CDesignator::CDesignator(designator_integration_msgs::Designator desigContent) {
  this->fillFromDesignatorMsg(desigContent);
}

CDesignator::CDesignator(enum DesignatorType edtType, CKeyValuePair* ckvpDescription) {
  if(ckvpDescription) {
    this->fillFromDescription(edtType, ckvpDescription->children());
  } else {
    this->setType(edtType);
  }
}

CDesignator::CDesignator(enum DesignatorType edtType, list<CKeyValuePair*> lstDescription) {
  this->fillFromDescription(edtType, lstDescription);
}

void CDesignator::fillFromDescription(enum DesignatorType edtType, list<CKeyValuePair*> lstDescription) {
  list<CKeyValuePair*> lstDescriptionNew;
  
  for(list<CKeyValuePair*>::iterator itKVP = lstDescription.begin();
      itKVP != lstDescription.end();
      itKVP++) {
    lstDescriptionNew.push_back((*itKVP)->copy());
  }
  
  this->setDescription(lstDescriptionNew);
  this->setType(edtType);
}

void CDesignator::setDescription(list<CKeyValuePair*> lstDescription) {
  m_lstChildren = lstDescription;
}

list<CKeyValuePair*> CDesignator::description() {
  return m_lstChildren;
}

void CDesignator::setType(enum DesignatorType edtType) {
  m_edtType = edtType;
  
  enum ValueType evtType = DESIGNATOR_OBJECT;
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
    
  default:
    break;
  }
  
  m_evtType = evtType;
}

void CDesignator::fillFromDesignatorMsg(designator_integration_msgs::Designator desigContent) {
  m_edtType = (enum DesignatorType)desigContent.type;
  
  list<CKeyValuePair*> lstKVPs;
  for(vector<designator_integration_msgs::KeyValuePair>::iterator itKVP = desigContent.description.begin();
      itKVP != desigContent.description.end();
      itKVP++) {
    designator_integration_msgs::KeyValuePair kvpCurrent = *itKVP;
    
    CKeyValuePair *ckvpCurrent = new CKeyValuePair(kvpCurrent);
    lstKVPs.push_back(ckvpCurrent);
  }
  
  while(lstKVPs.size() > 0) {
    for(list<CKeyValuePair*>::iterator itKVP = lstKVPs.begin();
	itKVP != lstKVPs.end();
	itKVP++) {
      CKeyValuePair *kvpCurrent = *itKVP;
      bool bFoundChild = false;
      
      for(list<CKeyValuePair*>::iterator itKVPInspect = lstKVPs.begin();
	  itKVPInspect != lstKVPs.end();
	  itKVPInspect++) {
	CKeyValuePair *kvpInspect = *itKVPInspect;
	
	if(kvpInspect != kvpCurrent && kvpInspect->parent() == kvpCurrent->id()) {
	  bFoundChild = true;
	  break;
	}
      }
      
      if(!bFoundChild) {
	bool bFoundParent = false;
	
	for(list<CKeyValuePair*>::iterator itKVPInspect = lstKVPs.begin();
	    itKVPInspect != lstKVPs.end();
	    itKVPInspect++) {
	  CKeyValuePair *kvpInspect = *itKVPInspect;
	  
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

enum DesignatorType CDesignator::type() {
  return m_edtType;
}

void CDesignator::printDesignator() {
  for(list<CKeyValuePair*>::iterator itKVP = m_lstChildren.begin();
      itKVP != m_lstChildren.end();
      itKVP++) {
    CKeyValuePair *kvpCurrent = *itKVP;
    
    kvpCurrent->printPair(0);
    cout << endl;
  }
}

designator_integration_msgs::Designator CDesignator::serializeToMessage() {
  designator_integration_msgs::Designator msgDesig;
  msgDesig.type = (int)m_edtType;
  
  int nHighestID = 0;
  vector<designator_integration_msgs::KeyValuePair> vecKVPs;
  for(list<CKeyValuePair*>::iterator itKVP = m_lstChildren.begin();
      itKVP != m_lstChildren.end();
      itKVP++) {
    CKeyValuePair *ckvpCurrent = *itKVP;
    
    vector<designator_integration_msgs::KeyValuePair> vecKVPMsgs = ckvpCurrent->serializeToMessage(0, nHighestID + 1);
    for(vector<designator_integration_msgs::KeyValuePair>::iterator itKVPMsg = vecKVPMsgs.begin();
	itKVPMsg != vecKVPMsgs.end();
	itKVPMsg++) {
      designator_integration_msgs::KeyValuePair kvpCurrent = *itKVPMsg;
      
      if(kvpCurrent.id > nHighestID) {
	nHighestID = kvpCurrent.id;
      }
      
      vecKVPs.push_back(kvpCurrent);
    }
  }
  
  msgDesig.description = vecKVPs;
  
  return msgDesig;
}
