#include <designators/CDesignator.h>


CDesignator::CDesignator() {
  this->m_edtType = UNKNOWN;
}

CDesignator::CDesignator(designator_integration_msgs::Designator desigContent) {
  this->fillFromDesignatorMsg(desigContent);
}

void CDesignator::setType(enum DesignatorType edtType) {
  m_edtType = edtType;
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
	  m_lstKeyValuePairs.push_back(kvpCurrent);
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
  for(list<CKeyValuePair*>::iterator itKVP = m_lstKeyValuePairs.begin();
      itKVP != m_lstKeyValuePairs.end();
      itKVP++) {
    CKeyValuePair *kvpCurrent = *itKVP;
    
    kvpCurrent->printPair(0);
  }
}

CKeyValuePair* CDesignator::addValue(string strKey) {
  CKeyValuePair *kvpNew = new CKeyValuePair(strKey);
  m_lstKeyValuePairs.push_back(kvpNew);
  
  return kvpNew;
}

CKeyValuePair* CDesignator::addValue(string strKey, string strValue) {
  CKeyValuePair *kvpNew = new CKeyValuePair(strKey, strValue);
  m_lstKeyValuePairs.push_back(kvpNew);
  
  return kvpNew;
}

CKeyValuePair* CDesignator::addValue(string strKey, float fValue) {
  CKeyValuePair *kvpNew = new CKeyValuePair(strKey, fValue);
  m_lstKeyValuePairs.push_back(kvpNew);
  
  return kvpNew;
}

CKeyValuePair* CDesignator::addValue(string strKey, geometry_msgs::PoseStamped psValue) {
  CKeyValuePair *kvpNew = new CKeyValuePair(strKey, psValue);
  m_lstKeyValuePairs.push_back(kvpNew);
  
  return kvpNew;
}

CKeyValuePair* CDesignator::keyValuePairForKey(string strKey) {
  CKeyValuePair *ckvpReturn = NULL;
  
  for(list<CKeyValuePair*>::iterator itKVP = m_lstKeyValuePairs.begin();
      itKVP != m_lstKeyValuePairs.end();
      itKVP++) {
    ckvpReturn = *itKVP;
  }
  
  return ckvpReturn;
}

designator_integration_msgs::Designator CDesignator::serializeToMessage() {
  designator_integration_msgs::Designator msgDesig;
  msgDesig.type = (int)m_edtType;
  
  int nHighestID = 0;
  vector<designator_integration_msgs::KeyValuePair> vecKVPs;
  for(list<CKeyValuePair*>::iterator itKVP = m_lstKeyValuePairs.begin();
      itKVP != m_lstKeyValuePairs.end();
      itKVP++) {
    CKeyValuePair *ckvpCurrent = *itKVP;
    
    vector<designator_integration_msgs::KeyValuePair> vecKVPMsgs = this->serializeKeyValuePair(ckvpCurrent, 0, nHighestID);
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

vector<designator_integration_msgs::KeyValuePair> CDesignator::serializeKeyValuePair(CKeyValuePair *ckvpSerialize, int nParent, int nHighestID) {
  vector<designator_integration_msgs::KeyValuePair> vecKVPs = ckvpSerialize->serializeToMessage(nParent, nHighestID + 1);
  
  return vecKVPs;
}

CKeyValuePair* CDesignator::getPairForKey(string strKey) {
  CKeyValuePair* ckvpReturn = NULL;
  for(list<CKeyValuePair*>::iterator itCKVP = m_lstKeyValuePairs.begin();
      itCKVP != m_lstKeyValuePairs.end();
      itCKVP++) {
    CKeyValuePair *ckvpCurrent = *itCKVP;
    
    if(strcasecmp(ckvpCurrent->key().c_str(), strKey.c_str()) == 0) {
      ckvpReturn = ckvpCurrent;
      break;
    }
  }
  
  return ckvpReturn;
}

string CDesignator::getStringValue(string strKey) {
  CKeyValuePair *ckvpCurrent = this->getPairForKey(strKey);
  
  if(ckvpCurrent) {
    return ckvpCurrent->stringValue();
  }
  
  return "";
}

float CDesignator::getFloatValue(string strKey) {
  CKeyValuePair *ckvpCurrent = this->getPairForKey(strKey);
  
  if(ckvpCurrent) {
    return ckvpCurrent->floatValue();
  }
  
  return 0.0;
}

geometry_msgs::PoseStamped CDesignator::getPoseStampedValue(string strKey) {
  CKeyValuePair *ckvpCurrent = this->getPairForKey(strKey);
  
  if(ckvpCurrent) {
    return ckvpCurrent->poseStampedValue();
  }
  
  geometry_msgs::PoseStamped psEmpty;
  return psEmpty;
}
