#include <designators/KeyValuePair.h>


namespace designator_integration {
  KeyValuePair::KeyValuePair() {
    this->init();
  }

  KeyValuePair::KeyValuePair(std::string strKey) {
    this->init();
    this->setKey(strKey);
  }

  KeyValuePair::KeyValuePair(std::string strKey, std::string strValue) {
    this->init();
    this->setKey(strKey);
    this->setValue(strValue);
  }

  KeyValuePair::KeyValuePair(std::string strKey, float fValue) {
    this->init();
    this->setKey(strKey);
    this->setValue(fValue);
  }

  KeyValuePair::KeyValuePair(std::string strKey, char* acValue, unsigned int unLength) {
    this->init();
    this->setKey(strKey);
    this->setValue(acValue, unLength);
  }

  KeyValuePair::KeyValuePair(designator_integration_msgs::KeyValuePair kvpContent) {
    this->init();
    this->fillWithKeyValueContent(kvpContent);
  }

  KeyValuePair::KeyValuePair(std::string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
    this->init();
    this->setKey(strKey);
    this->setValue(psPoseStampedValue);
  }

  KeyValuePair::KeyValuePair(std::string strKey, geometry_msgs::Pose posPoseValue) {
    this->init();
    this->setKey(strKey);
    this->setValue(posPoseValue);
  }

  KeyValuePair::KeyValuePair(std::list<KeyValuePair*> lstChildren) {
    this->init();
    this->setChildren(lstChildren);
  }

  KeyValuePair::~KeyValuePair() {
    this->clearDataValue();
  }

  void KeyValuePair::init() {
    m_nParent = 0;
    m_nID = 0;
    m_strValue = "";
    m_fValue = 0.0;
    m_evtType = STRING;
    m_acValue = NULL;
    m_unValueLength = 0;
    m_bIsAtom = false;
  }

  void KeyValuePair::fillWithKeyValueContent(designator_integration_msgs::KeyValuePair kvpContent) {
    m_nID = kvpContent.id;
    m_nParent = kvpContent.parent;
    m_evtType = (ValueType)kvpContent.type;
    
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
  
  bool KeyValuePair::isAtom() {
    return m_bIsAtom;
  }
  
  void KeyValuePair::setIsAtom(bool bIsAtom) {
    m_bIsAtom = bIsAtom;
  }
  
  KeyValuePair::ValueType KeyValuePair::type() {
    return m_evtType;
  }
  
  std::string KeyValuePair::stringValue() {
    // NOTE: Strings and floats can be returned as strings. Everything
    // else turns into a blank string.
    switch(m_evtType) {
    case STRING: {
      return m_strValue;
    } break;
    
    case FLOAT: {
      std::stringstream sts;
      sts << m_fValue;
      return sts.str();
    } break;
    
    default: {
      return "";
    } break;
    }
  }
  
  float KeyValuePair::floatValue() {
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
	std::cerr << "Error while converting '" << m_strValue << "' to float." << std::endl;
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

  char* KeyValuePair::dataValue() {
    return m_acValue;
  }

  unsigned int KeyValuePair::dataValueLength() {
    return m_unValueLength;
  }

  char* KeyValuePair::dataValue(unsigned int& unLength) {
    unLength = this->dataValueLength();
    
    return this->dataValue();
  }

  geometry_msgs::PoseStamped KeyValuePair::poseStampedValue() {
    return m_psPoseStampedValue;
  }

  geometry_msgs::Pose KeyValuePair::poseValue() {
    return m_posPoseValue;
  }

  int KeyValuePair::id() {
    return m_nID;
  }

  int KeyValuePair::parent() {
    return m_nParent;
  }

  std::string KeyValuePair::key() {
    return m_strKey;
  }

  KeyValuePair* KeyValuePair::addChild(std::string strKey, bool bAppendNew) {
    KeyValuePair* ckvpNewChild = NULL;
    
    if(!bAppendNew) {
      ckvpNewChild = this->childForKey(strKey);
    }
    
    if(!ckvpNewChild) {
      ckvpNewChild = new KeyValuePair();
      ckvpNewChild->setKey(strKey);
      this->addChild(ckvpNewChild);
    }
    
    return ckvpNewChild;
  }
  
  void KeyValuePair::addChild(KeyValuePair* ckvpChildAdd) {
    m_lstChildren.push_back(ckvpChildAdd);
    this->setType(LIST);
  }
  
  std::list<KeyValuePair*> KeyValuePair::children() {
    return m_lstChildren;
  }
  
  void KeyValuePair::setChildren(std::list<KeyValuePair*> lstChildren) {
    m_lstChildren = lstChildren;
  }
  
  void KeyValuePair::printPair(int nSpaceOffset, bool bOffsetRegular, bool bNewline) {
    std::cout << "(";
  
    if(!m_bIsAtom) {
      std::cout << m_strKey << " ";
    }
  
    switch(m_evtType) {
    case STRING: {
      std::cout << "\"" << m_strValue << "\")";
    } break;
    
    case FLOAT: {
      std::cout << m_fValue << ")";
    } break;
    
    case POSE: {
      std::cout << "[pose: [position: "
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
      std::cout << "[pose: "
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
      std::string strPrefix;
      
      // TODO(winkler): This check is SO wrong. Checks for the same
      // variable as the wrapping `switch'. This is bound to always
      // result in `LIST'. This is probably refering to the type of the
      // value's child node. Fix this.
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
      std::cout << "(";
      bool bFirst = true;
    
      for(int nI = 0; nI < m_lstChildren.size(); nI++) {
	// TODO(winkler): What is it we're doing here? Why not use a
	// simpler for(...) formulation? Figure out why this is being
	// done so complicated.
	std::list<KeyValuePair*>::iterator itChild = m_lstChildren.begin();
	advance(itChild, nI);
	KeyValuePair* ckvpChild = *itChild;
	
	if(!bFirst) {
	  this->printSpaces(nSpaceOffset + m_strKey.length() + 3);
	} else {
	  bFirst = false;
	}
      
	ckvpChild->printPair(nSpaceOffset + m_strKey.length() + 3);
      
	if(nI < m_lstChildren.size() - 1) {
	  std::cout << std::endl;
	}
      }
      
      std::cout << ")";
    } break;
    
    default:
      break;
    }
  }

  void KeyValuePair::printSpaces(int nSpaces) {
    for(int nI = 0; nI < nSpaces; nI++) {
      std::cout << " ";
    }
  }

  void KeyValuePair::setID(int nID) {
    m_nID = nID;
  }

  void KeyValuePair::setParent(int nParent) {
    m_nParent = nParent;
  }
  
  void KeyValuePair::addAtom(std::string strValue) {
    KeyValuePair* ckvpAtom = this->addChild("", strValue);
    ckvpAtom->setIsAtom(true);
  }
  
  void KeyValuePair::addAtom(float fValue) {
    KeyValuePair* ckvpAtom = this->addChild("", fValue);
    ckvpAtom->setIsAtom(true);
  }
  
  void KeyValuePair::addAtom(geometry_msgs::PoseStamped psPoseStampedValue) {
    KeyValuePair* ckvpAtom = this->addChild("", psPoseStampedValue);
    ckvpAtom->setIsAtom(true);
  }
  
  void KeyValuePair::addAtom(geometry_msgs::Pose psPoseValue) {
    KeyValuePair* ckvpAtom = this->addChild("", psPoseValue);
    ckvpAtom->setIsAtom(true);
  }
  
  void KeyValuePair::setValue(std::string strValue) {
    m_strValue = strValue;
    this->setType(STRING);
  }
  
  void KeyValuePair::setValue(float fValue) {
    m_fValue = fValue;
    this->setType(FLOAT);
  }
  
  void KeyValuePair::setValue(geometry_msgs::PoseStamped psPoseStampedValue) {
    m_psPoseStampedValue = psPoseStampedValue;
    this->setType(POSESTAMPED);
  }
  
  void KeyValuePair::setValue(geometry_msgs::Pose posPoseValue) {
    m_posPoseValue = posPoseValue;
    this->setType(POSE);
  }
  
  void KeyValuePair::clearDataValue() {
    if(m_acValue != NULL) {
      delete[] m_acValue;
      m_acValue = NULL;
      m_unValueLength = 0;
    }
  }
  
  void KeyValuePair::setValue(void* vdValue, unsigned int unLength) {
    this->clearDataValue();
    
    m_acValue = new char[unLength]();
    memcpy(m_acValue, vdValue, unLength);
    
    this->setType(DATA);
  }
  
  void KeyValuePair::setValue(std::string strKey, ValueType evtType, std::list<KeyValuePair*> lstDescription) {
    KeyValuePair* ckvpChild = this->childForKey(strKey);
    
    if(ckvpChild) {
      ckvpChild->setValue(evtType, lstDescription);
    } else {
      this->addChild(strKey, evtType, lstDescription);
    }
  }
  
  void KeyValuePair::setValue(ValueType evtType, std::list<KeyValuePair*> lstDescription) {
    m_lstChildren = lstDescription;
    this->setType(evtType);
  }
  
  KeyValuePair* KeyValuePair::addChild(std::string strKey, ValueType evtType, std::list<KeyValuePair*> lstDescription) {
    KeyValuePair* ckvpNewChild = this->addChild(strKey);
    ckvpNewChild->setValue(evtType, lstDescription);
    
    return ckvpNewChild;
  }
  
  void KeyValuePair::setLocationDesignatorDescription(std::string strKey, ValueType evtType, std::list<KeyValuePair*> lstDescription) {
    this->setValue(strKey, DESIGNATOR_LOCATION, lstDescription);
  }
  
  void KeyValuePair::setActionDesignatorDescription(std::string strKey, std::list<KeyValuePair*> lstDescription) {
    this->setValue(strKey, DESIGNATOR_ACTION, lstDescription);
  }
  
  void KeyValuePair::setObjectDesignatorDescription(std::string strKey, std::list<KeyValuePair*> lstDescription) {
    this->setValue(strKey, DESIGNATOR_OBJECT, lstDescription);
  }
  
  void KeyValuePair::setKey(std::string strKey) {
    m_strKey = strKey;
  }
  
  void KeyValuePair::setType(ValueType evtType) {
    m_evtType = evtType;
  }
  
  KeyValuePair* KeyValuePair::addChild(std::string strKey, std::string strValue) {
    KeyValuePair* ckvpNewChild = this->addChild(strKey);
    ckvpNewChild->setValue(strValue);
    
    return ckvpNewChild;
  }
  
  KeyValuePair* KeyValuePair::addChild(std::string strKey, float fValue) {
    KeyValuePair* ckvpNewChild = this->addChild(strKey);
    ckvpNewChild->setValue(fValue);
    
    return ckvpNewChild;
  }
  
  KeyValuePair* KeyValuePair::addChild(std::string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
    KeyValuePair* ckvpNewChild = this->addChild(strKey);
    ckvpNewChild->setValue(psPoseStampedValue);
    
    return ckvpNewChild;
  }
  
  KeyValuePair* KeyValuePair::addChild(std::string strKey, geometry_msgs::Pose posPoseValue) {
    KeyValuePair* ckvpNewChild = this->addChild(strKey);
    ckvpNewChild->setValue(posPoseValue);
    
    return ckvpNewChild;
  }
  
  KeyValuePair* KeyValuePair::addChild(std::string strKey, char *acValue, unsigned int unLength) {
    KeyValuePair* ckvpNewChild = this->addChild(strKey);
    ckvpNewChild->setValue(acValue, unLength);
    
    return ckvpNewChild;
  }
  
  std::vector<designator_integration_msgs::KeyValuePair> KeyValuePair::serializeToMessage(int nParent, int nID) {
    std::vector<designator_integration_msgs::KeyValuePair> vecReturn;
    designator_integration_msgs::KeyValuePair kvpSerialized;
    
    // Serialization header
    kvpSerialized.id = nID;
    kvpSerialized.parent = nParent;
    
    // Meta data
    kvpSerialized.key = m_strKey;
    kvpSerialized.type = (int)m_evtType;
    //kvpSerialized.is_atom = m_bIsAtom;
    
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
    for(KeyValuePair* kvpChild : m_lstChildren) {
      std::vector<designator_integration_msgs::KeyValuePair> vecChildren = kvpChild->serializeToMessage(nID, nHighestID + 1);
      
      for(designator_integration_msgs::KeyValuePair dikvpMsg : vecChildren) {
	if(dikvpMsg.id > nHighestID) {
	  nHighestID = dikvpMsg.id;
	}
	
	vecReturn.push_back(dikvpMsg);
      }
    }
    
    return vecReturn;
  }

  KeyValuePair* KeyValuePair::childForKey(std::string strKey) {
    KeyValuePair* kvpReturn = NULL;
    
    for(KeyValuePair* kvpCurrent : m_lstChildren) {
      if(strcasecmp(kvpCurrent->key().c_str(), strKey.c_str()) == 0) {
	kvpReturn = kvpCurrent;
	break;
      }
    }
  
    return kvpReturn;
  }

  bool KeyValuePair::removeChildForKey(std::string strKey) {
    bool bResult = false;
    
    KeyValuePair* ckvpChild = this->childForKey(strKey);
    if(ckvpChild) {
      m_lstChildren.remove(ckvpChild);
      delete ckvpChild;
    }
    
    return bResult;
  }
  
  std::string KeyValuePair::stringValue(std::string strChildKey) {
    KeyValuePair* ckvpChild = this->childForKey(strChildKey);
    
    if(ckvpChild) {
      return ckvpChild->stringValue();
    }
  
    return "";
  }
  
  float KeyValuePair::floatValue(std::string strChildKey) {
    KeyValuePair* ckvpChild = this->childForKey(strChildKey);
    
    if(ckvpChild) {
      return ckvpChild->floatValue();
    }
  
    return 0.0;
  }

  geometry_msgs::PoseStamped KeyValuePair::poseStampedValue(std::string strChildKey) {
    KeyValuePair* ckvpChild = this->childForKey(strChildKey);
    
    if(ckvpChild) {
      return ckvpChild->poseStampedValue();
    }
    
    geometry_msgs::PoseStamped psEmpty;
    return psEmpty;
  }
  
  geometry_msgs::Pose KeyValuePair::poseValue(std::string strChildKey) {
    KeyValuePair* ckvpChild = this->childForKey(strChildKey);
    
    if(ckvpChild) {
      return ckvpChild->poseValue();
    }
    
    geometry_msgs::Pose posEmpty;
    return posEmpty;
  }
  
  void KeyValuePair::setValue(std::string strKey, std::string strValue) {
    KeyValuePair* ckvpChild = this->childForKey(strKey);
  
    if(ckvpChild) {
      ckvpChild->setValue(strValue);
    } else {
      this->addChild(strKey, strValue);
    }
  }

  void KeyValuePair::setValue(std::string strKey, float fValue) {
    KeyValuePair* ckvpChild = this->childForKey(strKey);
  
    if(ckvpChild) {
      ckvpChild->setValue(fValue);
    } else {
      this->addChild(strKey, fValue);
    }
  }

  void KeyValuePair::setValue(std::string strKey, geometry_msgs::PoseStamped psPoseStampedValue) {
    KeyValuePair* ckvpChild = this->childForKey(strKey);
  
    if(ckvpChild) {
      ckvpChild->setValue(psPoseStampedValue);
    } else {
      this->addChild(strKey, psPoseStampedValue);
    }
  }

  void KeyValuePair::setValue(std::string strKey, geometry_msgs::Pose posPoseValue) {
    KeyValuePair* ckvpChild = this->childForKey(strKey);
  
    if(ckvpChild) {
      ckvpChild->setValue(posPoseValue);
    } else {
      this->addChild(strKey, posPoseValue);
    }
  }

  void KeyValuePair::clear() {
    m_lstChildren.clear();
  }

  KeyValuePair* KeyValuePair::copy() {
    KeyValuePair* ckvpCopy = new KeyValuePair();
  
    ckvpCopy->setValue(this->stringValue());
    ckvpCopy->setValue(this->floatValue());
    ckvpCopy->setValue(this->poseStampedValue());
    ckvpCopy->setValue(this->poseValue());
    ckvpCopy->setValue(this->dataValue(), this->dataValueLength());
    ckvpCopy->setType(this->type());
  
    ckvpCopy->setKey(this->key());
    ckvpCopy->setParent(this->parent());
    ckvpCopy->setID(this->id());
  
    for(KeyValuePair* kvpChild : m_lstChildren) {
      ckvpCopy->addChild(kvpChild->copy());
    }
    
    return ckvpCopy;
  }
  
  std::list<std::string> KeyValuePair::keys() {
    std::list<std::string> lstKeys;
    std::list<KeyValuePair*> lstChildren = this->children();
    
    for(KeyValuePair* kvpChild : lstChildren) {
      lstKeys.push_back(kvpChild->key());
    }
    
    return lstKeys;
  }
}
