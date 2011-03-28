/* Auto-generated by genmsg_cpp for file /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/srv/CoaxGetTrimMode.srv */
#ifndef COAX_MSGS_SERVICE_COAXGETTRIMMODE_H
#define COAX_MSGS_SERVICE_COAXGETTRIMMODE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"



#include "coax_msgs/CoaxTrimMode.h"

namespace coax_msgs
{
template <class ContainerAllocator>
struct CoaxGetTrimModeRequest_ : public ros::Message
{
  typedef CoaxGetTrimModeRequest_<ContainerAllocator> Type;

  CoaxGetTrimModeRequest_()
  {
  }

  CoaxGetTrimModeRequest_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxGetTrimModeRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "ebb2d5a0e1488ca7b436e3a43853c58b"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxGetTrimModeRequest
typedef  ::coax_msgs::CoaxGetTrimModeRequest_<std::allocator<void> > CoaxGetTrimModeRequest;

typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeRequest> CoaxGetTrimModeRequestPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeRequest const> CoaxGetTrimModeRequestConstPtr;


template <class ContainerAllocator>
struct CoaxGetTrimModeResponse_ : public ros::Message
{
  typedef CoaxGetTrimModeResponse_<ContainerAllocator> Type;

  CoaxGetTrimModeResponse_()
  : mode()
  {
  }

  CoaxGetTrimModeResponse_(const ContainerAllocator& _alloc)
  : mode(_alloc)
  {
  }

  typedef  ::coax_msgs::CoaxTrimMode_<ContainerAllocator>  _mode_type;
   ::coax_msgs::CoaxTrimMode_<ContainerAllocator>  mode;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxGetTrimModeResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "ebb2d5a0e1488ca7b436e3a43853c58b"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "ebb2d5a0e1488ca7b436e3a43853c58b"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "CoaxTrimMode mode\n\
\n\
\n\
================================================================================\n\
MSG: coax_msgs/CoaxTrimMode\n\
# Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */\n\
uint8 trimMode \n\
# Trim position for the roll axis */\n\
float32 rollTrim\n\
# Trim position for the pitch axis */\n\
float32 pitchTrim\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, mode);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, mode);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(mode);
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxGetTrimModeResponse
typedef  ::coax_msgs::CoaxGetTrimModeResponse_<std::allocator<void> > CoaxGetTrimModeResponse;

typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeResponse> CoaxGetTrimModeResponsePtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxGetTrimModeResponse const> CoaxGetTrimModeResponseConstPtr;

struct CoaxGetTrimMode
{

typedef CoaxGetTrimModeRequest Request;
typedef CoaxGetTrimModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CoaxGetTrimMode
} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxGetTrimModeRequest";
  }

  static const char* value(const  ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebb2d5a0e1488ca7b436e3a43853c58b";
  }

  static const char* value(const  ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xebb2d5a0e1488ca7ULL;
  static const uint64_t static_value2 = 0xb436e3a43853c58bULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxGetTrimModeResponse";
  }

  static const char* value(const  ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CoaxTrimMode mode\n\
\n\
\n\
================================================================================\n\
MSG: coax_msgs/CoaxTrimMode\n\
# Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */\n\
uint8 trimMode \n\
# Trim position for the roll axis */\n\
float32 rollTrim\n\
# Trim position for the pitch axis */\n\
float32 pitchTrim\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxGetTrimModeRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.mode);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxGetTrimModeResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<coax_msgs::CoaxGetTrimMode> {
  static const char* value() 
  {
    return "ebb2d5a0e1488ca7b436e3a43853c58b";
  }

  static const char* value(const coax_msgs::CoaxGetTrimMode&) { return value(); } 
};

template<>
struct DataType<coax_msgs::CoaxGetTrimMode> {
  static const char* value() 
  {
    return "coax_msgs/CoaxGetTrimMode";
  }

  static const char* value(const coax_msgs::CoaxGetTrimMode&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebb2d5a0e1488ca7b436e3a43853c58b";
  }

  static const char* value(const coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxGetTrimMode";
  }

  static const char* value(const coax_msgs::CoaxGetTrimModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebb2d5a0e1488ca7b436e3a43853c58b";
  }

  static const char* value(const coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxGetTrimMode";
  }

  static const char* value(const coax_msgs::CoaxGetTrimModeResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COAX_MSGS_SERVICE_COAXGETTRIMMODE_H

