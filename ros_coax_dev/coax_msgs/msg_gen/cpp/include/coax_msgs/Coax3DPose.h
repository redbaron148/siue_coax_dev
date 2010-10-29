/* Auto-generated by genmsg_cpp for file /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/msg/Coax3DPose.msg */
#ifndef COAX_MSGS_MESSAGE_COAX3DPOSE_H
#define COAX_MSGS_MESSAGE_COAX3DPOSE_H
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
struct Coax3DPose_ : public ros::Message
{
  typedef Coax3DPose_<ContainerAllocator> Type;

  Coax3DPose_()
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , yawrate(0.0)
  {
  }

  Coax3DPose_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , yawrate(0.0)
  {
  }

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _z_type;
  float z;

  typedef float _yawrate_type;
  float yawrate;


private:
  static const char* __s_getDataType_() { return "coax_msgs/Coax3DPose"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROSCPP_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "1ff9ebeac8f7c7a45d6bc0fd77076ef7"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROSCPP_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 x\n\
float32 y\n\
float32 z\n\
float32 yawrate\n\
\n\
\n\
"; }
public:
  ROSCPP_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROSCPP_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, z);
    ros::serialization::serialize(stream, yawrate);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, z);
    ros::serialization::deserialize(stream, yawrate);
    return stream.getData();
  }

  ROSCPP_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(z);
    size += ros::serialization::serializationLength(yawrate);
    return size;
  }

  typedef boost::shared_ptr< ::coax_msgs::Coax3DPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coax_msgs::Coax3DPose_<ContainerAllocator>  const> ConstPtr;
}; // struct Coax3DPose
typedef  ::coax_msgs::Coax3DPose_<std::allocator<void> > Coax3DPose;

typedef boost::shared_ptr< ::coax_msgs::Coax3DPose> Coax3DPosePtr;
typedef boost::shared_ptr< ::coax_msgs::Coax3DPose const> Coax3DPoseConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::coax_msgs::Coax3DPose_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::coax_msgs::Coax3DPose_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace coax_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::coax_msgs::Coax3DPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1ff9ebeac8f7c7a45d6bc0fd77076ef7";
  }

  static const char* value(const  ::coax_msgs::Coax3DPose_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1ff9ebeac8f7c7a4ULL;
  static const uint64_t static_value2 = 0x5d6bc0fd77076ef7ULL;
};

template<class ContainerAllocator>
struct DataType< ::coax_msgs::Coax3DPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "coax_msgs/Coax3DPose";
  }

  static const char* value(const  ::coax_msgs::Coax3DPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::coax_msgs::Coax3DPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 x\n\
float32 y\n\
float32 z\n\
float32 yawrate\n\
\n\
\n\
";
  }

  static const char* value(const  ::coax_msgs::Coax3DPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::coax_msgs::Coax3DPose_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::coax_msgs::Coax3DPose_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.yawrate);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Coax3DPose_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::coax_msgs::Coax3DPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::coax_msgs::Coax3DPose_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "yawrate: ";
    Printer<float>::stream(s, indent + "  ", v.yawrate);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COAX_MSGS_MESSAGE_COAX3DPOSE_H

