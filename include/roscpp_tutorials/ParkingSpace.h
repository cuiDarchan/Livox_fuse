// Generated by gencpp from file roscpp_tutorials/ParkingSpace.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_PARKINGSPACE_H
#define ROSCPP_TUTORIALS_MESSAGE_PARKINGSPACE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/Curved.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct ParkingSpace_
{
  typedef ParkingSpace_<ContainerAllocator> Type;

  ParkingSpace_()
    : id()
    , type(0)
    , polygon()
    , heading(0.0)
    , start_s(0.0)
    , end_s(0.0)  {
    }
  ParkingSpace_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , type(0)
    , polygon(_alloc)
    , heading(0.0)
    , start_s(0.0)
    , end_s(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef uint16_t _type_type;
  _type_type type;

   typedef  ::roscpp_tutorials::Curved_<ContainerAllocator>  _polygon_type;
  _polygon_type polygon;

   typedef float _heading_type;
  _heading_type heading;

   typedef float _start_s_type;
  _start_s_type start_s;

   typedef float _end_s_type;
  _end_s_type end_s;





  typedef boost::shared_ptr< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> const> ConstPtr;

}; // struct ParkingSpace_

typedef ::roscpp_tutorials::ParkingSpace_<std::allocator<void> > ParkingSpace;

typedef boost::shared_ptr< ::roscpp_tutorials::ParkingSpace > ParkingSpacePtr;
typedef boost::shared_ptr< ::roscpp_tutorials::ParkingSpace const> ParkingSpaceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dcca3e1c05618421ec7160fa11d7b0d1";
  }

  static const char* value(const ::roscpp_tutorials::ParkingSpace_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdcca3e1c05618421ULL;
  static const uint64_t static_value2 = 0xec7160fa11d7b0d1ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/ParkingSpace";
  }

  static const char* value(const ::roscpp_tutorials::ParkingSpace_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string id\n\
uint16 type\n\
Curved polygon\n\
float32 heading\n\
float32 start_s\n\
float32 end_s\n\
================================================================================\n\
MSG: roscpp_tutorials/Curved\n\
Vec3d[] points\n\
\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/Vec3d\n\
float32[3] point\n\
\n\
";
  }

  static const char* value(const ::roscpp_tutorials::ParkingSpace_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.polygon);
      stream.next(m.heading);
      stream.next(m.start_s);
      stream.next(m.end_s);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ParkingSpace_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::ParkingSpace_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::ParkingSpace_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.type);
    s << indent << "polygon: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::Curved_<ContainerAllocator> >::stream(s, indent + "  ", v.polygon);
    s << indent << "heading: ";
    Printer<float>::stream(s, indent + "  ", v.heading);
    s << indent << "start_s: ";
    Printer<float>::stream(s, indent + "  ", v.start_s);
    s << indent << "end_s: ";
    Printer<float>::stream(s, indent + "  ", v.end_s);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_PARKINGSPACE_H