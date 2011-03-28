/* Auto-generated by genmsg_cpp for file /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/srv/CoaxSetRawControl.srv */
#ifndef COAX_MSGS_SERVICE_COAXSETRAWCONTROL_H
#define COAX_MSGS_SERVICE_COAXSETRAWCONTROL_H
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
struct CoaxSetRawControlRequest_ : public ros::Message
{
  typedef CoaxSetRawControlRequest_<ContainerAllocator> Type;

  CoaxSetRawControlRequest_()
  : motor1(0.0)
  , motor2(0.0)
  , servo1(0.0)
  , servo2(0.0)
  {
  }

  CoaxSetRawControlRequest_(const ContainerAllocator& _alloc)
  : motor1(0.0)
  , motor2(0.0)
  , servo1(0.0)
  , servo2(0.0)
  {
  }

  typedef float _motor1_type;
  float motor1;

  typedef float _motor2_type;
  float motor2;

  typedef float _servo1_type;
  float servo1;

  typedef float _servo2_type;
  float servo2;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSetRawControlRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b66c77c051c7057221fbc455368c06bc"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "f5de27a093ebcc425d781d1a75a2f9d3"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 motor1\n\
float32 motor2\n\
float32 servo1\n\
float32 servo2\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, motor1);
    ros::serialization::serialize(stream, motor2);
    ros::serialization::serialize(stream, servo1);
    ros::serialization::serialize(stream, servo2);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, motor1);
    ros::serialization::deserialize(stream, motor2);
    ros::serialization::deserialize(stream, servo1);
    ros::serialization::deserialize(stream, servo2);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(motor1);
    size += ros::serialization::serializationLength(motor2);
    size += ros::serialization::serializationLength(servo1);
    size += ros::serialization::serializationLength(servo2);
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSetRawControlRequest
typedef  ::coax_msgs::CoaxSetRawControlRequest_<std::allocator<void> > CoaxSetRawControlRequest;

typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlRequest> CoaxSetRawControlRequestPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlRequest const> CoaxSetRawControlRequestConstPtr;


template <class ContainerAllocator>
struct CoaxSetRawControlResponse_ : public ros::Message
{
  typedef CoaxSetRawControlResponse_<ContainerAllocator> Type;

  CoaxSetRawControlResponse_()
  : result(0)
  {
  }

  CoaxSetRawControlResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int8_t _result_type;
  int8_t result;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSetRawControlResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "4414c67819626a1b8e0f043a9a0d6c9a"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "f5de27a093ebcc425d781d1a75a2f9d3"; }
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

  typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSetRawControlResponse
typedef  ::coax_msgs::CoaxSetRawControlResponse_<std::allocator<void> > CoaxSetRawControlResponse;

typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlResponse> CoaxSetRawControlResponsePtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSetRawControlResponse const> CoaxSetRawControlResponseConstPtr;

struct CoaxSetRawControl
{

typedef CoaxSetRawControlRequest Request;
typedef CoaxSetRawControlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CoaxSetRawControl
} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b66c77c051c7057221fbc455368c06bc";
  }

  static const char* value(const  ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb66c77c051c70572ULL;
  static const uint64_t static_value2 = 0x21fbc455368c06bcULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetRawControlRequest";
  }

  static const char* value(const  ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 motor1\n\
float32 motor2\n\
float32 servo1\n\
float32 servo2\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4414c67819626a1b8e0f043a9a0d6c9a";
  }

  static const char* value(const  ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4414c67819626a1bULL;
  static const uint64_t static_value2 = 0x8e0f043a9a0d6c9aULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetRawControlResponse";
  }

  static const char* value(const  ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 result\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.motor1);
    stream.next(m.motor2);
    stream.next(m.servo1);
    stream.next(m.servo2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSetRawControlRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSetRawControlResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<coax_msgs::CoaxSetRawControl> {
  static const char* value() 
  {
    return "f5de27a093ebcc425d781d1a75a2f9d3";
  }

  static const char* value(const coax_msgs::CoaxSetRawControl&) { return value(); } 
};

template<>
struct DataType<coax_msgs::CoaxSetRawControl> {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetRawControl";
  }

  static const char* value(const coax_msgs::CoaxSetRawControl&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5de27a093ebcc425d781d1a75a2f9d3";
  }

  static const char* value(const coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetRawControl";
  }

  static const char* value(const coax_msgs::CoaxSetRawControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5de27a093ebcc425d781d1a75a2f9d3";
  }

  static const char* value(const coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetRawControl";
  }

  static const char* value(const coax_msgs::CoaxSetRawControlResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COAX_MSGS_SERVICE_COAXSETRAWCONTROL_H

