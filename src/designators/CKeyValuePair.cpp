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

void CKeyValuePair::addChild(CKeyValuePair *kvpAdd) {
  m_lstChildren.push_back(kvpAdd);
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

void CKeyValuePair::setStringValue(string strValue) {
  m_strValue = strValue;
  this->setType(STRING);
}

void CKeyValuePair::setFloatValue(float fValue) {
  m_fValue = fValue;
  this->setType(FLOAT);
}

void CKeyValuePair::setPoseStampedValue(geometry_msgs::PoseStamped psPoseStampedValue) {
  m_psPoseStampedValue = psPoseStampedValue;
  this->setType(POSESTAMPED);
}

void CKeyValuePair::setValue(string strValue) {
  this->setStringValue(strValue);
}

void CKeyValuePair::setValue(float fValue) {
  this->setFloatValue(fValue);
}

void CKeyValuePair::setValue(geometry_msgs::PoseStamped psPoseStampedValue) {
  this->setPoseStampedValue(psPoseStampedValue);
}

void CKeyValuePair::setKey(string strKey) {
  m_strKey = strKey;
}

void CKeyValuePair::setType(enum ValueType evtType) {
  m_evtType = evtType;
}

CKeyValuePair *CKeyValuePair::addStringChild(string strKey, string strValue) {
  CKeyValuePair *ckvpNewChild = new CKeyValuePair();
  ckvpNewChild->setKey(strKey);
  ckvpNewChild->setStringValue(strValue);
  this->addChild(ckvpNewChild);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addFloatChild(string strKey, float fValue) {
  CKeyValuePair *ckvpNewChild = new CKeyValuePair();
  ckvpNewChild->setKey(strKey);
  ckvpNewChild->setFloatValue(fValue);
  this->addChild(ckvpNewChild);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addPoseStampedChild(string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
  CKeyValuePair *ckvpNewChild = new CKeyValuePair();
  ckvpNewChild->setKey(strKey);
  ckvpNewChild->setPoseStampedValue(psPoseStampedValue);
  this->addChild(ckvpNewChild);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey) {
  CKeyValuePair *ckvpNewChild = new CKeyValuePair();
  ckvpNewChild->setKey(strKey);
  this->addChild(ckvpNewChild);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, string strValue) {
  return this->addStringChild(strKey, strValue);
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, float fValue) {
  return this->addFloatChild(strKey, fValue);
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
  return this->addPoseStampedChild(strKey, psPoseStampedValue);
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
