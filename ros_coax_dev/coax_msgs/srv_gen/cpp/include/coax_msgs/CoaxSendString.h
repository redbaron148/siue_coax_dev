/* Auto-generated by genmsg_cpp for file /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/srv/CoaxSendString.srv */
#ifndef COAX_MSGS_SERVICE_COAXSENDSTRING_H
#define COAX_MSGS_SERVICE_COAXSENDSTRING_H
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
struct CoaxSendStringRequest_ : public ros::Message
{
  typedef CoaxSendStringRequest_<ContainerAllocator> Type;

  CoaxSendStringRequest_()
  : text()
  {
  }

  CoaxSendStringRequest_(const ContainerAllocator& _alloc)
  : text(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _text_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  text;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSendStringRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "74697ed3d931f6eede8bf3a8dfeca160"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "05354734935e371f83dc4d09f1c13d77"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string text\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, text);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, text);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(text);
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSendStringRequest
typedef  ::coax_msgs::CoaxSendStringRequest_<std::allocator<void> > CoaxSendStringRequest;

typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringRequest> CoaxSendStringRequestPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringRequest const> CoaxSendStringRequestConstPtr;


template <class ContainerAllocator>
struct CoaxSendStringResponse_ : public ros::Message
{
  typedef CoaxSendStringResponse_<ContainerAllocator> Type;

  CoaxSendStringResponse_()
  : result(0)
  {
  }

  CoaxSendStringResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int8_t _result_type;
  int8_t result;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSendStringResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "4414c67819626a1b8e0f043a9a0d6c9a"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "05354734935e371f83dc4d09f1c13d77"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 result\n\
\n\
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

  typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSendStringResponse
typedef  ::coax_msgs::CoaxSendStringResponse_<std::allocator<void> > CoaxSendStringResponse;

typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringResponse> CoaxSendStringResponsePtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSendStringResponse const> CoaxSendStringResponseConstPtr;

struct CoaxSendString
{

typedef CoaxSendStringRequest Request;
typedef CoaxSendStringResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CoaxSendString
} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "74697ed3d931f6eede8bf3a8dfeca160";
  }

  static const char* value(const  ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x74697ed3d931f6eeULL;
  static const uint64_t static_value2 = 0xde8bf3a8dfeca160ULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSendStringRequest";
  }

  static const char* value(const  ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string text\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4414c67819626a1b8e0f043a9a0d6c9a";
  }

  static const char* value(const  ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4414c67819626a1bULL;
  static const uint64_t static_value2 = 0x8e0f043a9a0d6c9aULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSendStringResponse";
  }

  static const char* value(const  ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 result\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSendStringRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.text);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSendStringRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSendStringResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSendStringResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<coax_msgs::CoaxSendString> {
  static const char* value() 
  {
    return "05354734935e371f83dc4d09f1c13d77";
  }

  static const char* value(const coax_msgs::CoaxSendString&) { return value(); } 
};

template<>
struct DataType<coax_msgs::CoaxSendString> {
  static const char* value() 
  {
    return "coax_msgs/CoaxSendString";
  }

  static const char* value(const coax_msgs::CoaxSendString&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSendStringRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "05354734935e371f83dc4d09f1c13d77";
  }

  static const char* value(const coax_msgs::CoaxSendStringRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSendStringRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSendString";
  }

  static const char* value(const coax_msgs::CoaxSendStringRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "05354734935e371f83dc4d09f1c13d77";
  }

  static const char* value(const coax_msgs::CoaxSendStringResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSendStringResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSendString";
  }

  static const char* value(const coax_msgs::CoaxSendStringResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COAX_MSGS_SERVICE_COAXSENDSTRING_H

