#include <designators/Designator.h>

#if DESIGNATOR_WITH_JSON
#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <rapidjson/reader.h>
#endif // DESIGNATOR_WITH_JSON


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
    setType((DesignatorType)desigContent.type);
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

  designator_integration_msgs::Designator Designator::serializeToMessage() {
    designator_integration_msgs::Designator msgDesig;
    msgDesig.type = (int)m_edtType;

    int nHighestID = 0;
    std::vector<designator_integration_msgs::KeyValuePair> vecKVPs;

    for(KeyValuePair* kvpCurrent : m_lstChildren) {
      std::vector<designator_integration_msgs::KeyValuePair> vecKVPMsgs = kvpCurrent->serializeToMessage(0, nHighestID + 1);
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

#if DESIGNATOR_WITH_JSON
#define TYPE_FIELD "_designator_type"

class JSON2Designator {
public:
  std::vector<KeyValuePair*> kvStack;
  std::vector<std::list<KeyValuePair*>> listStask;
  std::vector<char> array;
  geometry_msgs::PoseStamped pose;
  std::string lastKey;

  enum{
    READ_KEY = 0,
    READ_VALUE,
    READ_TYPE,
    READ_ARRAY,
    READ_POSESTAMPED,
    READ_POSE
  } status;

  JSON2Designator(Designator *designator) : status(READ_KEY)
  {
    kvStack.push_back(designator);
  }

  template<class T>
  bool Number(T n)
  {
    if(status == READ_TYPE)
    {
      KeyValuePair *kv = kvStack.back();
      KeyValuePair::ValueType vt = (KeyValuePair::ValueType)n;
      kv->setType(vt);
      if(vt == KeyValuePair::POSE)
        status = READ_POSE;
      else if(vt == KeyValuePair::POSESTAMPED)
        status = READ_POSESTAMPED;
      else
        status = READ_KEY;
    }
    else if(status == READ_ARRAY)
    {
      array.push_back((char)n);
    }
    else if(status == READ_POSE || status == READ_POSESTAMPED)
    {
      if(lastKey == "pos_x")
        pose.pose.position.x = (double)n;
      else if(lastKey == "pos_y")
        pose.pose.position.y = (double)n;
      else if(lastKey == "pos_z")
        pose.pose.position.z = (double)n;
      else if(lastKey == "rot_x")
        pose.pose.orientation.x = (double)n;
      else if(lastKey == "rot_y")
        pose.pose.orientation.y = (double)n;
      else if(lastKey == "rot_z")
        pose.pose.orientation.z = (double)n;
      else if(lastKey == "rot_w")
        pose.pose.orientation.w = (double)n;
      else if(lastKey == "seq")
        pose.header.seq = (uint32_t)n;
      else if(lastKey == "stamp")
        pose.header.stamp.fromNSec((uint64_t)n);
      else
        return false;
    }
    else
    {
      KeyValuePair *kv = kvStack.back();
      kv->setValue((double)n);
      if(kv->key().empty())
        kv->setIsAtom(true);
      kvStack.pop_back();
      status = READ_KEY;
    }
    return true;
  }

  bool Null()
  {
    return false;
  }

  bool Bool(bool b)
  {
    return false;
  }

  bool Int(int i)
  {
    return Number<int32_t>(i);
  }

  bool Uint(unsigned u)
  {
    return Number<uint32_t>(u);
  }

  bool Int64(int64_t i)
  {
    return Number<int64_t>(i);
  }

  bool Uint64(uint64_t u)
  {
    return Number<uint64_t>(u);
  }

  bool Double(double d)
  {
    return Number<double>(d);
  }

  bool String(const char *str, rapidjson::SizeType length, bool copy)
  {
    std::string value(str, length);

    if(status == READ_POSESTAMPED)
    {
      if(lastKey == "frame_id")
        pose.header.frame_id = value;
      else
        return false;
    }
    else
    {
      KeyValuePair *kv = kvStack.back();
      kv->setValue(value);
      if(kv->key().empty())
        kv->setIsAtom(true);
      kvStack.pop_back();
      status = READ_KEY;
    }
    return true;
  }

  bool Key(const char *str, rapidjson::SizeType length, bool copy)
  {
    std::string value(str, length);

    lastKey = value;
    if(lastKey ==  TYPE_FIELD)
      status = READ_TYPE;
    if(status ==  READ_KEY){
      status = READ_VALUE;
      KeyValuePair *parent = kvStack.back();
      KeyValuePair::ValueType t = parent->type();
      KeyValuePair *child = parent->addChild(lastKey, true);
      parent->setType(t);
      kvStack.push_back(child);
    }
    return true;
  }

  bool StartObject()
  {
    status = READ_KEY;
    return true;
  }

  bool EndObject(rapidjson::SizeType memberCount)
  {
    KeyValuePair *kv = kvStack.back();
    if(status == READ_POSE)
    {
      kv->setValue(pose.pose);
    }
    else if(status == READ_POSESTAMPED)
    {
      kv->setValue(pose);
    }
    if(kv->key().empty())
      kv->setIsAtom(true);
    kvStack.pop_back();
    status = READ_KEY;
    return true;
  }

  bool StartArray()
  {
    status = READ_ARRAY;
    return true;
  }

  bool EndArray(rapidjson::SizeType elementCount)
  {
    KeyValuePair *kv = kvStack.back();
    kv->setValue(array.data(), array.size());
    if(kv->key().empty())
      kv->setIsAtom(true);
    kvStack.pop_back();
    array.clear();
    status = READ_KEY;
    return true;
  }
};

  void Designator::fillFromJSON(const std::string &json)
  {
    JSON2Designator handler(this);
    rapidjson::Reader reader;
    rapidjson::StringStream ss(json.c_str());
    rapidjson::ParseResult res = reader.Parse(ss, handler);
    if(!res)
    {
      std::cout << "Parse error: " << res.Code() << " at " << res.Offset() << std::endl;
    }

    switch(m_evtType)
    {
      case DESIGNATOR_ACTION:
        this->m_edtType = ACTION;
        break;
      case DESIGNATOR_OBJECT:
        this->m_edtType = OBJECT;
        break;
      case DESIGNATOR_HUMAN:
        this->m_edtType = HUMAN;
        break;
      case DESIGNATOR_LOCATION:
        this->m_edtType = LOCATION;
        break;
    }
    return;
  }

class Designator2JSON
{
public:
  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> json;

  Designator2JSON() : json(s)
  {
  }

  void serializePose(geometry_msgs::Pose pose)
  {
    json.String("pos_x");
    json.Double(pose.position.x);
    json.String("pos_y");
    json.Double(pose.position.y);
    json.String("pos_z");
    json.Double(pose.position.z);
    json.String("rot_x");
    json.Double(pose.orientation.x);
    json.String("rot_y");
    json.Double(pose.orientation.y);
    json.String("rot_z");
    json.Double(pose.orientation.z);
    json.String("rot_w");
    json.Double(pose.orientation.w);
  }

  void serializeHeader(std_msgs::Header header)
  {
    json.String("seq");
    json.Uint(header.seq);
    json.String("frame_id");
    json.String(header.frame_id);
    json.String("stamp");
    json.Uint64(header.stamp.toNSec());
  }

  void serializeList(const int type, std::list<KeyValuePair*> children)
  {
    json.StartObject();
    json.String(TYPE_FIELD);
    json.Int(type);
    for(KeyValuePair* kvpCurrent : children)
    {
      serializeToJSON(kvpCurrent);
    }
    json.EndObject();
  }

  void serializeToJSON(KeyValuePair *child, bool first = false)
  {
    if(!first)
      json.String(child->key());
    switch(child->type()){
    case KeyValuePair::STRING:
      json.String(child->stringValue());
      break;
    case KeyValuePair::FLOAT:
      json.Double(child->floatValue());
      break;
    case KeyValuePair::DATA:
    {
      std::cout << "DATA" << std::endl;
      uint32_t length;
      char *data = child->dataValue(length);
      json.StartArray();
      for(size_t i = 0; i < length; ++i)
      {
        json.Int((int)data[i]);
      }
      json.EndArray();
      break;
    }
    case KeyValuePair::LIST:
      serializeList(KeyValuePair::LIST, child->children());
      break;
    case KeyValuePair::POSESTAMPED:
      json.StartObject();
      json.String(TYPE_FIELD);
      json.Int(KeyValuePair::POSESTAMPED);
      serializeHeader(child->poseStampedValue().header);
      serializePose(child->poseStampedValue().pose);
      json.EndObject();
      break;
    case KeyValuePair::POSE:
      json.StartObject();
      json.String(TYPE_FIELD);
      json.Int(KeyValuePair::POSE);
      serializePose(child->poseValue());
      json.EndObject();
      break;
    case KeyValuePair::DESIGNATOR_ACTION:
      serializeList(KeyValuePair::DESIGNATOR_ACTION, child->children());
      break;
    case KeyValuePair::DESIGNATOR_OBJECT:
      serializeList(KeyValuePair::DESIGNATOR_OBJECT, child->children());
      break;
    case KeyValuePair::DESIGNATOR_LOCATION:
      serializeList(KeyValuePair::DESIGNATOR_LOCATION, child->children());
      break;
    case KeyValuePair::DESIGNATOR_HUMAN:
      serializeList(KeyValuePair::DESIGNATOR_HUMAN, child->children());
      break;
    }
  }
};

  std::string Designator::serializeToJSON()
  {
    Designator2JSON handler;
    handler.serializeToJSON(this, true);
    return std::string(handler.s.GetString());
  }

  std::string Designator::serializeToJSON(std::vector<Designator> &designators)
  {
    Designator2JSON handler;
    handler.json.StartArray();
    for(size_t i = 0; i < designators.size(); ++i)
    {
      handler.serializeToJSON(&designators[i], true);
    }
    handler.json.EndArray();
    return std::string(handler.s.GetString());
  }
#endif // DESIGNATOR_WITH_JSON
}
