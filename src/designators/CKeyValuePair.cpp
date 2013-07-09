#include <designators/CKeyValuePair.h>


CKeyValuePair::CKeyValuePair() {
  this->init();
}

CKeyValuePair::CKeyValuePair(string strKey) {
  this->init();
  this->setKey(strKey);
}

CKeyValuePair::CKeyValuePair(string strKey, string strValue) {
  this->init();
  this->setKey(strKey);
  this->setValue(strValue);
}

CKeyValuePair::CKeyValuePair(string strKey, float fValue) {
  this->init();
  this->setKey(strKey);
  this->setValue(fValue);
}

CKeyValuePair::CKeyValuePair(designator_integration_msgs::KeyValuePair kvpContent) {
  this->init();
  this->fillWithKeyValueContent(kvpContent);
}

CKeyValuePair::CKeyValuePair(string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
  this->init();
  this->setKey(strKey);
  this->setValue(psPoseStampedValue);
}

CKeyValuePair::~CKeyValuePair() {
  if(m_acData != NULL) {
    delete[] m_acData;
  }
}

void CKeyValuePair::init() {
  m_nParent = 0;
  m_nID = 0;
  m_strValue = "";
  m_fValue = 0.0;
  m_evtType = STRING;
  m_acData = NULL;
}

void CKeyValuePair::fillWithKeyValueContent(designator_integration_msgs::KeyValuePair kvpContent) {
  m_nID = kvpContent.id;
  m_nParent = kvpContent.parent;
  m_evtType = (enum ValueType)kvpContent.type;
  
  m_strKey = kvpContent.key;
  m_strValue = kvpContent.value_string;
  m_fValue = kvpContent.value_float;
  m_psPoseStampedValue = kvpContent.value_posestamped;
  // Add 'data' field here as well.
}

enum ValueType CKeyValuePair::type() {
  return m_evtType;
}

string CKeyValuePair::stringValue() {
  return m_strValue;
}

float CKeyValuePair::floatValue() {
  return m_fValue;
}

geometry_msgs::PoseStamped CKeyValuePair::poseStampedValue() {
  return m_psPoseStampedValue;
}

int CKeyValuePair::id() {
  return m_nID;
}

int CKeyValuePair::parent() {
  return m_nParent;
}

string CKeyValuePair::key() {
  return m_strKey;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey) {
  CKeyValuePair *ckvpNewChild = this->childForKey(strKey);
  
  if(!ckvpNewChild) {
    ckvpNewChild = new CKeyValuePair();
    ckvpNewChild->setKey(strKey);
    this->addChild(ckvpNewChild);
  }
  
  return ckvpNewChild;
}

void CKeyValuePair::addChild(CKeyValuePair *ckvpChildAdd) {
  m_lstChildren.push_back(ckvpChildAdd);
  this->setType(LIST);
}

list<CKeyValuePair*> CKeyValuePair::children() {
  return m_lstChildren;
}

void CKeyValuePair::printPair(int nSpaceOffset, bool bOffsetRegular, bool bNewline) {
  switch(m_evtType) {
  case STRING: {
    if(bOffsetRegular) {
      this->printSpaces(nSpaceOffset);
    }
    cout << "(" << m_strKey << " " << m_strValue << ")";
    if(bNewline) {
      cout << endl;
    }
  } break;

  case FLOAT: {
    if(bOffsetRegular) {
      this->printSpaces(nSpaceOffset);
    }
    cout << "(" << m_strKey << " " << m_fValue << ")";
    if(bNewline) {
      cout << endl;
    }
  } break;

  case POSESTAMPED: {
    // print this
  } break;

  case DATA: {
    if(bOffsetRegular) {
      this->printSpaces(nSpaceOffset);
    }
    cout << "(" << m_strKey << " " << "[data])";
    if(bNewline) {
      cout << endl;
    }
  } break;

  case LIST: {
    this->printSpaces(nSpaceOffset + 1);
    cout << "(" << m_strKey << endl;
    this->printSpaces(nSpaceOffset + 2);
    
    for(list<CKeyValuePair*>::iterator itChild = m_lstChildren.begin();
	itChild != m_lstChildren.end();
	itChild++) {
      CKeyValuePair *kvpChild = *itChild;
      list<CKeyValuePair*>::iterator itNext = itChild++;
      bool bLast = (itNext == m_lstChildren.end());
      itChild--;
      
      bool bFirst = (itChild == m_lstChildren.begin());
      
      if(itChild == m_lstChildren.begin()) {
	cout << "(";
      }
      
      kvpChild->printPair(nSpaceOffset + 3, !bFirst, !bLast);
    }
    
    cout << ")" << endl;
  } break;
  }
}

void CKeyValuePair::printSpaces(int nSpaces) {
  for(int nI = 0; nI < nSpaces; nI++) {
    cout << " ";
  }
}

void CKeyValuePair::setID(int nID) {
  m_nID = nID;
}

void CKeyValuePair::setParent(int nParent) {
  m_nParent = nParent;
}

void CKeyValuePair::setValue(string strValue) {
  m_strValue = strValue;
  this->setType(STRING);
}

void CKeyValuePair::setValue(float fValue) {
  m_fValue = fValue;
  this->setType(FLOAT);
}

void CKeyValuePair::setValue(geometry_msgs::PoseStamped psPoseStampedValue) {
  m_psPoseStampedValue = psPoseStampedValue;
  this->setType(POSESTAMPED);
}

void CKeyValuePair::setKey(string strKey) {
  m_strKey = strKey;
}

void CKeyValuePair::setType(enum ValueType evtType) {
  m_evtType = evtType;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, string strValue) {
  CKeyValuePair *ckvpNewChild = this->addChild(strKey);
  ckvpNewChild->setValue(strValue);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, float fValue) {
  CKeyValuePair *ckvpNewChild = this->addChild(strKey);
  ckvpNewChild->setValue(fValue);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
  CKeyValuePair *ckvpNewChild = this->addChild(strKey);
  ckvpNewChild->setValue(psPoseStampedValue);
  
  return ckvpNewChild;
}

vector<designator_integration_msgs::KeyValuePair> CKeyValuePair::serializeToMessage(int nParent, int nID) {
  vector<designator_integration_msgs::KeyValuePair> vecReturn;
  designator_integration_msgs::KeyValuePair kvpSerialized;
  
  // Serialization header
  kvpSerialized.id = nID;
  kvpSerialized.parent = nParent;
  
  // Meta data
  kvpSerialized.key = m_strKey;
  kvpSerialized.type = (int)m_evtType;
  
  // Values
  kvpSerialized.value_string = m_strValue;
  kvpSerialized.value_float = m_fValue;
  kvpSerialized.value_posestamped = m_psPoseStampedValue;
  
  vecReturn.push_back(kvpSerialized);
  
  int nHighestID = nID;
  for(list<CKeyValuePair*>::iterator itChild = m_lstChildren.begin();
      itChild != m_lstChildren.end();
      itChild++) {
    vector<designator_integration_msgs::KeyValuePair> vecChildren = (*itChild)->serializeToMessage(nID, nHighestID + 1);
    
    for(vector<designator_integration_msgs::KeyValuePair>::iterator itMsg = vecChildren.begin();
	itMsg != vecChildren.end();
	itMsg++) {
      designator_integration_msgs::KeyValuePair kvpChild = *itMsg;
      
      if(kvpChild.id > nHighestID) {
	nHighestID = kvpChild.id;
      }
      
      vecReturn.push_back(kvpChild);
    }
  }
  
  return vecReturn;
}

CKeyValuePair* CKeyValuePair::childForKey(string strKey) {
  CKeyValuePair *ckvpReturn = NULL;
  
  for(list<CKeyValuePair*>::iterator itKVP = m_lstChildren.begin();
      itKVP != m_lstChildren.end();
      itKVP++) {
    CKeyValuePair *ckvpCurrent = *itKVP;
    
    if(strcasecmp(ckvpCurrent->key().c_str(), strKey.c_str()) == 0) {
      ckvpReturn = ckvpCurrent;
      break;
    }
  }
  
  return ckvpReturn;
}

string CKeyValuePair::stringValue(string strChildKey) {
  CKeyValuePair *ckvpChild = this->childForKey(strChildKey);
  
  if(ckvpChild) {
    return ckvpChild->stringValue();
  }
  
  return "";
}

float CKeyValuePair::floatValue(string strChildKey) {
  CKeyValuePair *ckvpChild = this->childForKey(strChildKey);
  
  if(ckvpChild) {
    return ckvpChild->floatValue();
  }
  
  return 0.0;
}

void CKeyValuePair::setValue(string strKey, string strValue) {
  CKeyValuePair *ckvpChild = this->childForKey(strKey);
  
  if(ckvpChild) {
    ckvpChild->setValue(strValue);
  } else {
    this->addChild(strKey, strValue);
  }
}

void CKeyValuePair::setValue(string strKey, float fValue) {
  CKeyValuePair *ckvpChild = this->childForKey(strKey);
  
  if(ckvpChild) {
    ckvpChild->setValue(fValue);
  } else {
    this->addChild(strKey, fValue);
  }
}

void CKeyValuePair::setValue(string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
  CKeyValuePair *ckvpChild = this->childForKey(strKey);
  
  if(ckvpChild) {
    ckvpChild->setValue(psPoseStampedValue);
  } else {
    this->addChild(strKey, psPoseStampedValue);
  }
}
