/* Auto-generated by genmsg_cpp for file /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/msg/CoaxRawControl.msg */
#ifndef COAX_MSGS_MESSAGE_COAXRAWCONTROL_H
#define COAX_MSGS_MESSAGE_COAXRAWCONTROL_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace coax_msgs
{
template <class ContainerAllocator>
struct CoaxRawControl_ : public ros::Message
{
  typedef CoaxRawControl_<ContainerAllocator> Type;

  CoaxRawControl_()
  : motor1(0.0)
  , motor2(0.0)
  , servo1(0.0)
  , servo2(0.0)
  {
  }

  CoaxRawControl_(const ContainerAllocator& _alloc)
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
  static const char* __s_getDataType_() { return "coax_msgs/CoaxRawControl"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b66c77c051c7057221fbc455368c06bc"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 motor1\n\
float32 motor2\n\
float32 servo1\n\
float32 servo2\n\
\n\
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

  typedef boost::shared_ptr< ::coax_msgs::CoaxRawControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxRawControl_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxRawControl
typedef  ::coax_msgs::CoaxRawControl_<std::allocator<void> > CoaxRawControl;

typedef boost::shared_ptr< ::coax_msgs::CoaxRawControl> CoaxRawControlPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxRawControl const> CoaxRawControlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::coax_msgs::CoaxRawControl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::coax_msgs::CoaxRawControl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxRawControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b66c77c051c7057221fbc455368c06bc";
  }

  static const char* value(const  ::coax_msgs::CoaxRawControl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb66c77c051c70572ULL;
  static const uint64_t static_value2 = 0x21fbc455368c06bcULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxRawControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxRawControl";
  }

  static const char* value(const  ::coax_msgs::CoaxRawControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxRawControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 motor1\n\
float32 motor2\n\
float32 servo1\n\
float32 servo2\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxRawControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxRawControl_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxRawControl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.motor1);
    stream.next(m.motor2);
    stream.next(m.servo1);
    stream.next(m.servo2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxRawControl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::coax_msgs::CoaxRawControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::coax_msgs::CoaxRawControl_<ContainerAllocator> & v) 
  {
    s << indent << "motor1: ";
    Printer<float>::stream(s, indent + "  ", v.motor1);
    s << indent << "motor2: ";
    Printer<float>::stream(s, indent + "  ", v.motor2);
    s << indent << "servo1: ";
    Printer<float>::stream(s, indent + "  ", v.servo1);
    s << indent << "servo2: ";
    Printer<float>::stream(s, indent + "  ", v.servo2);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COAX_MSGS_MESSAGE_COAXRAWCONTROL_H
