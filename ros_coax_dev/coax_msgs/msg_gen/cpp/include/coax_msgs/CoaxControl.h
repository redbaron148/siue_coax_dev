/* Auto-generated by genmsg_cpp for file /home/coax/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/msg/CoaxControl.msg */
#ifndef COAX_MSGS_MESSAGE_COAXCONTROL_H
#define COAX_MSGS_MESSAGE_COAXCONTROL_H
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
struct CoaxControl_ : public ros::Message
{
  typedef CoaxControl_<ContainerAllocator> Type;

  CoaxControl_()
  : roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , altitude(0.0)
  {
  }

  CoaxControl_(const ContainerAllocator& _alloc)
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
  static const char* __s_getDataType_() { return "coax_msgs/CoaxControl"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "fb03125013e0c3e7f30559de7a629a22"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

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

  typedef boost::shared_ptr< ::coax_msgs::CoaxControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::CoaxControl_<ContainerAllocator>  const> ConstPtr;
}; // struct CoaxControl
typedef  ::coax_msgs::CoaxControl_<std::allocator<void> > CoaxControl;

typedef boost::shared_ptr< ::coax_msgs::CoaxControl> CoaxControlPtr;
typedef boost::shared_ptr< ::coax_msgs::CoaxControl const> CoaxControlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::coax_msgs::CoaxControl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::coax_msgs::CoaxControl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::CoaxControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fb03125013e0c3e7f30559de7a629a22";
  }

  static const char* value(const  ::coax_msgs::CoaxControl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfb03125013e0c3e7ULL;
  static const uint64_t static_value2 = 0xf30559de7a629a22ULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::CoaxControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/CoaxControl";
  }

  static const char* value(const  ::coax_msgs::CoaxControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::CoaxControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 roll\n\
float32 pitch\n\
float32 yaw\n\
float32 altitude\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::CoaxControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::CoaxControl_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::CoaxControl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.altitude);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CoaxControl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::coax_msgs::CoaxControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::coax_msgs::CoaxControl_<ContainerAllocator> & v) 
  {
    s << indent << "roll: ";
    Printer<float>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<float>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "altitude: ";
    Printer<float>::stream(s, indent + "  ", v.altitude);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COAX_MSGS_MESSAGE_COAXCONTROL_H

