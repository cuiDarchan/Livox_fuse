// Generated by gencpp from file roscpp_tutorials/Lanemarkers.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_LANEMARKERS_H
#define ROSCPP_TUTORIALS_MESSAGE_LANEMARKERS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/MsgHeader.h>
#include <roscpp_tutorials/Lanemarker.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct Lanemarkers_
{
  typedef Lanemarkers_<ContainerAllocator> Type;

  Lanemarkers_()
    : header()
    , sensor_id()
    , measured_timestamp()
    , lanemarkers()
    , lanemarker_num(0)  {
    }
  Lanemarkers_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sensor_id(_alloc)
    , measured_timestamp()
    , lanemarkers(_alloc)
    , lanemarker_num(0)  {
  (void)_alloc;
    }



   typedef  ::roscpp_tutorials::MsgHeader_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _sensor_id_type;
  _sensor_id_type sensor_id;

   typedef ros::Time _measured_timestamp_type;
  _measured_timestamp_type measured_timestamp;

   typedef std::vector< ::roscpp_tutorials::Lanemarker_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::roscpp_tutorials::Lanemarker_<ContainerAllocator> >::other >  _lanemarkers_type;
  _lanemarkers_type lanemarkers;

   typedef uint8_t _lanemarker_num_type;
  _lanemarker_num_type lanemarker_num;





  typedef boost::shared_ptr< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> const> ConstPtr;

}; // struct Lanemarkers_

typedef ::roscpp_tutorials::Lanemarkers_<std::allocator<void> > Lanemarkers;

typedef boost::shared_ptr< ::roscpp_tutorials::Lanemarkers > LanemarkersPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::Lanemarkers const> LanemarkersConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'roscpp_tutorials': ['/home/cui-dell/catkin_ws/src/roscpp_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c5d8486c671b829c13f08523755dd08";
  }

  static const char* value(const ::roscpp_tutorials::Lanemarkers_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c5d8486c671b829ULL;
  static const uint64_t static_value2 = 0xc13f08523755dd08ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/Lanemarkers";
  }

  static const char* value(const ::roscpp_tutorials::Lanemarkers_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MsgHeader header\n\
string sensor_id\n\
time measured_timestamp\n\
Lanemarker[] lanemarkers\n\
uint8 lanemarker_num\n\
================================================================================\n\
MSG: roscpp_tutorials/MsgHeader\n\
uint64 timestamp\n\
uint64 sequence_num\n\
uint16 module_name\n\
uint16 status\n\
uint64 is_debag\n\
uint64 measured_timestamp\n\
uint8[3] version\n\
uint64 token\n\
uint64 token_timestamp\n\
string detail\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Lanemarker\n\
LaneParam image_lane\n\
\n\
LaneParam world_lane\n\
\n\
uint16 type\n\
\n\
float32 type_confidence\n\
\n\
float32 speed_limit\n\
\n\
uint16 spatial\n\
\n\
uint32 track_id\n\
================================================================================\n\
MSG: roscpp_tutorials/LaneParam\n\
float32 cx_quality\n\
    \n\
float32 start\n\
    \n\
float32 end\n\
    \n\
float32 c0\n\
    \n\
float32 c1\n\
    \n\
float32 c2\n\
    \n\
float32 c3\n\
\n\
Point3d[] samples\n\
    \n\
float32 samples_confidence\n\
================================================================================\n\
MSG: roscpp_tutorials/Point3d\n\
	float32 x  # in meters or m/s\n\
	float32 y  # in meters or m/s\n\
	float32 z  # height in meters or m/s\n\
";
  }

  static const char* value(const ::roscpp_tutorials::Lanemarkers_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sensor_id);
      stream.next(m.measured_timestamp);
      stream.next(m.lanemarkers);
      stream.next(m.lanemarker_num);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Lanemarkers_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::Lanemarkers_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::Lanemarkers_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::MsgHeader_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sensor_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sensor_id);
    s << indent << "measured_timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.measured_timestamp);
    s << indent << "lanemarkers[]" << std::endl;
    for (size_t i = 0; i < v.lanemarkers.size(); ++i)
    {
      s << indent << "  lanemarkers[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::roscpp_tutorials::Lanemarker_<ContainerAllocator> >::stream(s, indent + "    ", v.lanemarkers[i]);
    }
    s << indent << "lanemarker_num: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.lanemarker_num);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_LANEMARKERS_H