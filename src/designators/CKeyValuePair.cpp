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

CKeyValuePair::CKeyValuePair(string strKey, char *acValue, unsigned int unLength) {
  this->init();
  this->setKey(strKey);
  this->setValue(acValue, unLength);
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

CKeyValuePair::CKeyValuePair(string strKey, geometry_msgs::Pose posPoseValue) {
  this->init();
  this->setKey(strKey);
  this->setValue(posPoseValue);
}

CKeyValuePair::~CKeyValuePair() {
  this->clearDataValue();
}

void CKeyValuePair::init() {
  m_nParent = 0;
  m_nID = 0;
  m_strValue = "";
  m_fValue = 0.0;
  m_evtType = STRING;
  m_acValue = NULL;
  m_unValueLength = 0;
}

void CKeyValuePair::fillWithKeyValueContent(designator_integration_msgs::KeyValuePair kvpContent) {
  m_nID = kvpContent.id;
  m_nParent = kvpContent.parent;
  m_evtType = (enum ValueType)kvpContent.type;
  
  m_strKey = kvpContent.key;
  m_strValue = kvpContent.value_string;
  m_fValue = kvpContent.value_float;
  m_psPoseStampedValue = kvpContent.value_posestamped;
  m_posPoseValue = kvpContent.value_pose;
  
  m_unValueLength = kvpContent.value_data.size();
  if(m_unValueLength > 0) {
    m_acValue = new char[m_unValueLength]();
    for(int nI = 0; nI < m_unValueLength; nI++) {
      m_acValue[nI] = kvpContent.value_data[nI];
    }
  }
}

enum ValueType CKeyValuePair::type() {
  return m_evtType;
}

string CKeyValuePair::stringValue() {
  // NOTE: Strings and floats can be returned as strings. Everything
  // else turns into a blank string.
  switch(m_evtType) {
  case STRING: {
    return m_strValue;
  } break;
    
  case FLOAT: {
    stringstream sts;
    sts << m_fValue;
    return sts.str();
  } break;
    
  default: {
    return "";
  } break;
  }
}

float CKeyValuePair::floatValue() {
  // NOTE: Strings and floats can be returned as floats. Everything
  // else turns into the float value 0.0f. Strings that are returned
  // as floats must be valid number literals. If this is not the case,
  // an error message will be printed and the value 0.0f will be
  // returned.
  switch(m_evtType) {
  case STRING: {
    float fValue = 0.0f;
    if(sscanf(m_strValue.c_str(), "%f", &fValue) == EOF) {
      // Something went wrong.
      cerr << "Error while converting '" << m_strValue << "' to float." << endl;
      return 0.0f;
    } else {
      return fValue;
    }
  } break;
    
  case FLOAT: {
    return m_fValue;
  } break;
    
  default: {
    return 0.0f;
  } break;
  }
}

char *CKeyValuePair::dataValue() {
  return m_acValue;
}

unsigned int CKeyValuePair::dataValueLength() {
  return m_unValueLength;
}

char *CKeyValuePair::dataValue(unsigned int &unLength) {
  unLength = this->dataValueLength();
  
  return this->dataValue();
}

geometry_msgs::PoseStamped CKeyValuePair::poseStampedValue() {
  return m_psPoseStampedValue;
}

geometry_msgs::Pose CKeyValuePair::poseValue() {
  return m_posPoseValue;
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

CKeyValuePair *CKeyValuePair::addChild(string strKey, bool bAppendNew) {
  CKeyValuePair *ckvpNewChild = NULL;
  
  if(!bAppendNew) {
    ckvpNewChild = this->childForKey(strKey);
  }
  
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
  cout << "(" << m_strKey << " ";
  
  switch(m_evtType) {
  case STRING: {
    cout << m_strValue << ")";
  } break;
    
  case FLOAT: {
    cout << m_fValue << ")";
  } break;
    
  case POSE: {
    cout << "[pose: [position: "
	 << m_posPoseValue.position.x << ", "
	 << m_posPoseValue.position.y << ", "
	 << m_posPoseValue.position.z << "], "
	 << "[orientation: "
	 << m_posPoseValue.orientation.x << ", "
	 << m_posPoseValue.orientation.y << ", "
	 << m_posPoseValue.orientation.z << ", "
	 << m_posPoseValue.orientation.w << "]])";
  } break;
    
  case POSESTAMPED: {
    cout << "[pose: "
	 << "[stamp: " << m_psPoseStampedValue.header.stamp.toSec() << "], "
	 << "[frame-id: " << m_psPoseStampedValue.header.frame_id << "], "
	 << "[position: "
	 << m_psPoseStampedValue.pose.position.x << ", "
	 << m_psPoseStampedValue.pose.position.y << ", "
	 << m_psPoseStampedValue.pose.position.z << "], "
	 << "[orientation: "
	 << m_psPoseStampedValue.pose.orientation.x << ", "
	 << m_psPoseStampedValue.pose.orientation.y << ", "
	 << m_psPoseStampedValue.pose.orientation.z << ", "
	 << m_psPoseStampedValue.pose.orientation.w << "]])";
  } break;
    
  case DESIGNATOR_ACTION:
  case DESIGNATOR_OBJECT:
  case DESIGNATOR_LOCATION:
  case LIST: {
    string strPrefix;
    switch(m_evtType) {
    case DESIGNATOR_ACTION:
      strPrefix = "<action (";
      break;
    case DESIGNATOR_OBJECT:
      strPrefix = "<object (";
      break;
    case DESIGNATOR_LOCATION:
      strPrefix = "<location (";
      break;
    case LIST:
      strPrefix = "(";
      break;
    }
    cout << "(";
    bool bFirst = true;
    
    for(int nI = 0; nI < m_lstChildren.size(); nI++) {
      list<CKeyValuePair*>::iterator itChild = m_lstChildren.begin();
      advance(itChild, nI);
      CKeyValuePair *ckvpChild = *itChild;
      
      if(!bFirst) {
	this->printSpaces(nSpaceOffset + m_strKey.length() + 3);
      } else {
	bFirst = false;
      }
      
      ckvpChild->printPair(nSpaceOffset + m_strKey.length() + 3);
      
      if(nI < m_lstChildren.size() - 1) {
	cout << endl;
      }
    }
    
    cout << ")";
  } break;
    
  default:
    break;
  }
  
  // switch(m_evtType) {
  // case STRING: {
  //   if(bOffsetRegular) {
  //     this->printSpaces(nSpaceOffset);
  //   }
  //   cout << "(" << m_strKey << " " << m_strValue << ")";
  //   if(bNewline) {
  //     cout << endl;
  //   }
  // } break;

  // case FLOAT: {
  //   if(bOffsetRegular) {
  //     this->printSpaces(nSpaceOffset);
  //   }
  //   cout << "(" << m_strKey << " " << m_fValue << ")";
  //   if(bNewline) {
  //     cout << endl;
  //   }
  // } break;

  // case POSESTAMPED: {
  //   // print this
  // } break;

  // case DATA: {
  //   if(bOffsetRegular) {
  //     this->printSpaces(nSpaceOffset);
  //   }
  //   cout << "(" << m_strKey << " " << "[data])";
  //   if(bNewline) {
  //     cout << endl;
  //   }
  // } break;

  // case LIST: {
  //   this->printSpaces(nSpaceOffset + 1);
  //   cout << "(" << m_strKey << endl;
  //   this->printSpaces(nSpaceOffset + 2);
    
  //   for(list<CKeyValuePair*>::iterator itChild = m_lstChildren.begin();
  // 	itChild != m_lstChildren.end();
  // 	itChild++) {
  //     CKeyValuePair *kvpChild = *itChild;
  //     list<CKeyValuePair*>::iterator itNext = itChild++;
  //     bool bLast = (itNext == m_lstChildren.end());
  //     itChild--;
      
  //     bool bFirst = (itChild == m_lstChildren.begin());
      
  //     if(itChild == m_lstChildren.begin()) {
  // 	cout << "(";
  //     }
      
  //     kvpChild->printPair(nSpaceOffset + 3, !bFirst, !bLast);
  //   }
    
  //   cout << ")" << endl;
  // } break;
  // }
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

void CKeyValuePair::setValue(geometry_msgs::Pose posPoseValue) {
  m_posPoseValue = posPoseValue;
  this->setType(POSE);
}

void CKeyValuePair::clearDataValue() {
  if(m_acValue != NULL) {
    delete[] m_acValue;
    m_acValue = NULL;
    m_unValueLength = 0;
  }
}

void CKeyValuePair::setValue(char *acValue, unsigned int unLength) {
  this->clearDataValue();
  
  m_acValue = new char[unLength]();
  memcpy(m_acValue, acValue, unLength);
  
  this->setType(DATA);
}

void CKeyValuePair::setValue(string strKey, enum ValueType evtType, list<CKeyValuePair*> lstDescription) {
  CKeyValuePair *ckvpChild = this->childForKey(strKey);
  
  if(ckvpChild) {
    ckvpChild->setValue(evtType, lstDescription);
  } else {
    this->addChild(strKey, evtType, lstDescription);
  }
}

void CKeyValuePair::setValue(enum ValueType evtType, list<CKeyValuePair*> lstDescription) {
  m_lstChildren = lstDescription;
  this->setType(evtType);
}

CKeyValuePair* CKeyValuePair::addChild(string strKey, enum ValueType evtType, list<CKeyValuePair*> lstDescription) {
  CKeyValuePair *ckvpNewChild = this->addChild(strKey);
  ckvpNewChild->setValue(evtType, lstDescription);
  
  return ckvpNewChild;
}

void CKeyValuePair::setLocationDesignatorDescription(string strKey, enum ValueType evtType, list<CKeyValuePair*> lstDescription) {
  this->setValue(strKey, DESIGNATOR_LOCATION, lstDescription);
}

void CKeyValuePair::setActionDesignatorDescription(string strKey, list<CKeyValuePair*> lstDescription) {
  this->setValue(strKey, DESIGNATOR_ACTION, lstDescription);
}

void CKeyValuePair::setObjectDesignatorDescription(string strKey, list<CKeyValuePair*> lstDescription) {
  this->setValue(strKey, DESIGNATOR_OBJECT, lstDescription);
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

CKeyValuePair *CKeyValuePair::addChild(string strKey, geometry_msgs::Pose posPoseValue) {
  CKeyValuePair *ckvpNewChild = this->addChild(strKey);
  ckvpNewChild->setValue(posPoseValue);
  
  return ckvpNewChild;
}

CKeyValuePair *CKeyValuePair::addChild(string strKey, char *acValue, unsigned int unLength) {
  CKeyValuePair *ckvpNewChild = this->addChild(strKey);
  ckvpNewChild->setValue(acValue, unLength);
  
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
  kvpSerialized.value_pose = m_posPoseValue;
  
  for(int nI = 0; nI < m_unValueLength; nI++) {
    kvpSerialized.value_data.push_back(m_acValue[nI]);
  }
  
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

geometry_msgs::PoseStamped CKeyValuePair::poseStampedValue(string strChildKey) {
  CKeyValuePair *ckvpChild = this->childForKey(strChildKey);
  
  if(ckvpChild) {
    return ckvpChild->poseStampedValue();
  }
  
  geometry_msgs::PoseStamped psEmpty;
  return psEmpty;
}

geometry_msgs::Pose CKeyValuePair::poseValue(string strChildKey) {
  CKeyValuePair *ckvpChild = this->childForKey(strChildKey);
  
  if(ckvpChild) {
    return ckvpChild->poseValue();
  }
  
  geometry_msgs::Pose posEmpty;
  return posEmpty;
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

void CKeyValuePair::setValue(string strKey, geometry_msgs::Pose posPoseValue) {
  CKeyValuePair *ckvpChild = this->childForKey(strKey);
  
  if(ckvpChild) {
    ckvpChild->setValue(posPoseValue);
  } else {
    this->addChild(strKey, posPoseValue);
  }
}

void CKeyValuePair::clear() {
  m_lstChildren.clear();
}

CKeyValuePair* CKeyValuePair::copy() {
  CKeyValuePair *ckvpCopy = new CKeyValuePair();
  
  ckvpCopy->setValue(this->stringValue());
  ckvpCopy->setValue(this->floatValue());
  ckvpCopy->setValue(this->poseStampedValue());
  ckvpCopy->setValue(this->poseValue());
  ckvpCopy->setValue(this->dataValue(), this->dataValueLength());
  ckvpCopy->setType(this->type());
  
  ckvpCopy->setKey(this->key());
  ckvpCopy->setParent(this->parent());
  ckvpCopy->setID(this->id());
  
  for(list<CKeyValuePair*>::iterator itChild = m_lstChildren.begin();
      itChild != m_lstChildren.end();
      itChild++) {
    ckvpCopy->addChild((*itChild)->copy());
  }
  
  return ckvpCopy;
}

list<string> CKeyValuePair::keys() {
  list<string> lstKeys;
  list<CKeyValuePair*> lstChildren = this->children();
  
  for(list<CKeyValuePair*>::iterator itPair = lstChildren.begin();
      itPair != lstChildren.end();
      itPair++) {
    lstKeys.push_back((*itPair)->key());
  }
  
  return lstKeys;
}
