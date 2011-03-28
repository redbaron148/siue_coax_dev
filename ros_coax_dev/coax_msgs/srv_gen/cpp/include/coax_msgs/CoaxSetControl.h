/* Auto-generated by genmsg_cpp for file /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/srv/CoaxSetControl.srv */
#ifndef COAX_MSGS_SERVICE_COAXSETCONTROL_H
#define COAX_MSGS_SERVICE_COAXSETCONTROL_H
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
struct CoaxSetControlRequest_ : public ros::Message
{
  typedef CoaxSetControlRequest_<ContainerAllocator> Type;

  CoaxSetControlRequest_()
  : roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , altitude(0.0)
  {
  }

  CoaxSetControlRequest_(const ContainerAllocator& _alloc)
  : roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , altitude(0.0)
  {
  }

  typedef float _roll_type;
  float roll;

  typedef float _pitch_type;
  float pitch;

  typedef float _yaw_type;
  float yaw;

  typedef float _altitude_type;
  float altitude;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSetControlRequest"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "fb03125013e0c3e7f30559de7a629a22"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "3462c8168747cd838501461513fced34"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 roll\n\
float32 pitch\n\
float32 yaw\n\
float32 altitude\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, roll);
    ros::serialization::serialize(stream, pitch);
    ros::serialization::serialize(stream, yaw);
    ros::serialization::serialize(stream, altitude);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, roll);
    ros::serialization::deserialize(stream, pitch);
    ros::serialization::deserialize(stream, yaw);
    ros::serialization::deserialize(stream, altitude);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(roll);
    size += ros::serialization::serializationLength(pitch);
    size += ros::serialization::serializationLength(yaw);
    size += ros::serialization::serializationLength(altitude);
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSetControlRequest
typedef  ::coax_msgs::CoaxSetControlRequest_<std::allocator<void> > CoaxSetControlRequest;

typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlRequest> CoaxSetControlRequestPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlRequest const> CoaxSetControlRequestConstPtr;


template <class ContainerAllocator>
struct CoaxSetControlResponse_ : public ros::Message
{
  typedef CoaxSetControlResponse_<ContainerAllocator> Type;

  CoaxSetControlResponse_()
  : result(0)
  {
  }

  CoaxSetControlResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int8_t _result_type;
  int8_t result;


private:
  static const char* __s_getDataType_() { return "coax_msgs/CoaxSetControlResponse"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "4414c67819626a1b8e0f043a9a0d6c9a"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "3462c8168747cd838501461513fced34"; }
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

  typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxSetControlResponse
typedef  ::coax_msgs::CoaxSetControlResponse_<std::allocator<void> > CoaxSetControlResponse;

typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlResponse> CoaxSetControlResponsePtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxSetControlResponse const> CoaxSetControlResponseConstPtr;

struct CoaxSetControl
{

typedef CoaxSetControlRequest Request;
typedef CoaxSetControlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CoaxSetControl
} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fb03125013e0c3e7f30559de7a629a22";
  }

  static const char* value(const  ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfb03125013e0c3e7ULL;
  static const uint64_t static_value2 = 0xf30559de7a629a22ULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetControlRequest";
  }

  static const char* value(const  ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 roll\n\
float32 pitch\n\
float32 yaw\n\
float32 altitude\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4414c67819626a1b8e0f043a9a0d6c9a";
  }

  static const char* value(const  ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4414c67819626a1bULL;
  static const uint64_t static_value2 = 0x8e0f043a9a0d6c9aULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetControlResponse";
  }

  static const char* value(const  ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 result\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSetControlRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.altitude);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSetControlRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxSetControlResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxSetControlResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<coax_msgs::CoaxSetControl> {
  static const char* value() 
  {
    return "3462c8168747cd838501461513fced34";
  }

  static const char* value(const coax_msgs::CoaxSetControl&) { return value(); } 
};

template<>
struct DataType<coax_msgs::CoaxSetControl> {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetControl";
  }

  static const char* value(const coax_msgs::CoaxSetControl&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3462c8168747cd838501461513fced34";
  }

  static const char* value(const coax_msgs::CoaxSetControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSetControlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetControl";
  }

  static const char* value(const coax_msgs::CoaxSetControlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3462c8168747cd838501461513fced34";
  }

  static const char* value(const coax_msgs::CoaxSetControlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<coax_msgs::CoaxSetControlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxSetControl";
  }

  static const char* value(const coax_msgs::CoaxSetControlResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COAX_MSGS_SERVICE_COAXSETCONTROL_H

