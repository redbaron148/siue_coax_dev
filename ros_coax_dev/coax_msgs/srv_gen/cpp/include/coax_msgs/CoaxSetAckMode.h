/* Auto-generated by genmsg_cpp for file /home/coax/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/srv/CoaxSetAckMode.srv */
#ifndef COAX_MSGS_SERVICE_COAXSETACKMODE_H
#define COAX_MSGS_SERVICE_COAXSETACKMODE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace coax_msgs
{
template <class ContainerAllocator>
struct CoaxSetAckModeRequest_ : public ros::Message
{
  typedef CoaxSetAckModeRequest_<ContainerAllocator> Type;

  CoaxSetAckModeRequest_()
  : mode(0)
  {
  }

  CoaxSetAckModeRequest_(const ContainerAllocator& _alloc)
  : mode(0)
  {
  }

  typedef uint8_t _mode_type;
  uint8_t mode;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSetAckModeRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "89b81386720be1cd0ce7a3953fcd1b19"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "4cc5eeb93a9b2000ef0e38bcc7d41dcc"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "uint8 mode\n\
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

  typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSetAckModeRequest
typedef  ::coax_msgs::CoaxSetAckModeRequest_<std::allocator<void> > CoaxSetAckModeRequest;

typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeRequest> CoaxSetAckModeRequestPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeRequest const> CoaxSetAckModeRequestConstPtr;


template <class ContainerAllocator>
struct CoaxSetAckModeResponse_ : public ros::Message
{
  typedef CoaxSetAckModeResponse_<ContainerAllocator> Type;

  CoaxSetAckModeResponse_()
  : result(0)
  {
  }

  CoaxSetAckModeResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int8_t _result_type;
  int8_t result;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSetAckModeResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "4414c67819626a1b8e0f043a9a0d6c9a"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "4cc5eeb93a9b2000ef0e38bcc7d41dcc"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 result\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, result);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, result);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(result);
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSetAckModeResponse
typedef  ::coax_msgs::CoaxSetAckModeResponse_<std::allocator<void> > CoaxSetAckModeResponse;

typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeResponse> CoaxSetAckModeResponsePtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSetAckModeResponse const> CoaxSetAckModeResponseConstPtr;

struct CoaxSetAckMode
{

typedef CoaxSetAckModeRequest Request;
typedef CoaxSetAckModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CoaxSetAckMode
} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "89b81386720be1cd0ce7a3953fcd1b19";
  }

  static const char* value(const  ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x89b81386720be1cdULL;
  static const uint64_t static_value2 = 0x0ce7a3953fcd1b19ULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetAckModeRequest";
  }

  static const char* value(const  ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 mode\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4414c67819626a1b8e0f043a9a0d6c9a";
  }

  static const char* value(const  ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4414c67819626a1bULL;
  static const uint64_t static_value2 = 0x8e0f043a9a0d6c9aULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetAckModeResponse";
  }

  static const char* value(const  ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.mode);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSetAckModeRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSetAckModeResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<coax_msgs::CoaxSetAckMode> {
  static const char* value() 
  {
    return "4cc5eeb93a9b2000ef0e38bcc7d41dcc";
  }

  static const char* value(const coax_msgs::CoaxSetAckMode&) { return value(); } 
};

template<>
struct DataType<coax_msgs::CoaxSetAckMode> {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetAckMode";
  }

  static const char* value(const coax_msgs::CoaxSetAckMode&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4cc5eeb93a9b2000ef0e38bcc7d41dcc";
  }

  static const char* value(const coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetAckMode";
  }

  static const char* value(const coax_msgs::CoaxSetAckModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4cc5eeb93a9b2000ef0e38bcc7d41dcc";
  }

  static const char* value(const coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetAckMode";
  }

  static const char* value(const coax_msgs::CoaxSetAckModeResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COAX_MSGS_SERVICE_COAXSETACKMODE_H

