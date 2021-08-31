// Generated by gencpp from file roscpp_tutorials/Debug.msg
// DO NOT EDIT!


#ifndef ROSCPP_TUTORIALS_MESSAGE_DEBUG_H
#define ROSCPP_TUTORIALS_MESSAGE_DEBUG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roscpp_tutorials/SimpleLonDebug.h>
#include <roscpp_tutorials/SimpleLatDebug.h>

namespace roscpp_tutorials
{
template <class ContainerAllocator>
struct Debug_
{
  typedef Debug_<ContainerAllocator> Type;

  Debug_()
    : simple_lon_debug()
    , simple_lat_debug()  {
    }
  Debug_(const ContainerAllocator& _alloc)
    : simple_lon_debug(_alloc)
    , simple_lat_debug(_alloc)  {
  (void)_alloc;
    }



   typedef  ::roscpp_tutorials::SimpleLonDebug_<ContainerAllocator>  _simple_lon_debug_type;
  _simple_lon_debug_type simple_lon_debug;

   typedef  ::roscpp_tutorials::SimpleLatDebug_<ContainerAllocator>  _simple_lat_debug_type;
  _simple_lat_debug_type simple_lat_debug;





  typedef boost::shared_ptr< ::roscpp_tutorials::Debug_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roscpp_tutorials::Debug_<ContainerAllocator> const> ConstPtr;

}; // struct Debug_

typedef ::roscpp_tutorials::Debug_<std::allocator<void> > Debug;

typedef boost::shared_ptr< ::roscpp_tutorials::Debug > DebugPtr;
typedef boost::shared_ptr< ::roscpp_tutorials::Debug const> DebugConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roscpp_tutorials::Debug_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roscpp_tutorials::Debug_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roscpp_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'roscpp_tutorials': ['/home/cui-dell/catkin_ws/src/roscpp_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Debug_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roscpp_tutorials::Debug_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Debug_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roscpp_tutorials::Debug_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Debug_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roscpp_tutorials::Debug_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roscpp_tutorials::Debug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0c2cd3e7890cdf79489f200fb785dbf1";
  }

  static const char* value(const ::roscpp_tutorials::Debug_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0c2cd3e7890cdf79ULL;
  static const uint64_t static_value2 = 0x489f200fb785dbf1ULL;
};

template<class ContainerAllocator>
struct DataType< ::roscpp_tutorials::Debug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roscpp_tutorials/Debug";
  }

  static const char* value(const ::roscpp_tutorials::Debug_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roscpp_tutorials::Debug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SimpleLonDebug simple_lon_debug\n\
SimpleLatDebug simple_lat_debug\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/SimpleLonDebug\n\
float32 station_reference\n\
float32 station_error\n\
float32 station_error_limited\n\
float32 preview_station_error\n\
float32 speed_reference\n\
float32 speed_error\n\
float32 speed_controller_input_limited\n\
float32 preview_speed_reference\n\
float32 preview_speed_error\n\
float32 preview_acceleration_reference\n\
float32 acceleration_cmd_closeloop\n\
float32 acceleration_cmd\n\
float32 acceleration_lookup\n\
float32 speed_lookup\n\
float32 calibration_value\n\
float32 throttle_cmd\n\
float32 brake_cmd\n\
bool is_full_stop\n\
float32 slope_offset_compensation\n\
float32 current_station\n\
float32 path_remain\n\
float32 auto_model_time\n\
float32 gear_position\n\
bool is_common_stop\n\
float32 current_acc\n\
float32 filtered_acc\n\
float32 lon_controller_status\n\
float32 common_stop_time\n\
float32 common_stop_s\n\
float32 matched_lon_curvature\n\
float32 matched_lon_station\n\
float32 preview_lon_station\n\
float32 preview_lon_curvature\n\
float32 reference_lon_curvature  \n\
float32 acceleration_cmd_feedforward\n\
float32 acceleration_cmd_compensation\n\
float32 acceleration_cmd_correction\n\
\n\
================================================================================\n\
MSG: roscpp_tutorials/SimpleLatDebug\n\
float32 lateral_error\n\
float32 ref_heading\n\
float32 heading\n\
float32 heading_error\n\
float32 heading_error_rate\n\
float32 lateral_error_rate\n\
float32 curvature\n\
float32 steer_angle\n\
float32 steer_angle_feedforward\n\
float32 steer_angle_lateral_contribution\n\
float32 steer_angle_lateral_rate_contribution\n\
float32 steer_angle_heading_contribution\n\
float32 steer_angle_heading_rate_contribution\n\
float32 steer_angle_feedback\n\
float32 steering_position\n\
float32 ref_speed\n\
float32 steer_angle_limited\n\
float32 steer_angle_integral\n\
float32 steer_angle_correct\n\
float32 steer_angle_roll_compensate\n\
float32 ref_x\n\
float32 ref_y\n\
float32 real_speed\n\
float32 matrix_k1\n\
float32 matrix_k2\n\
float32 matrix_k3\n\
float32 matrix_k4\n\
float32 matched_lat_acceleration\n\
float32 matched_lat_speed\n\
float32 matched_lat_station\n\
";
  }

  static const char* value(const ::roscpp_tutorials::Debug_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roscpp_tutorials::Debug_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.simple_lon_debug);
      stream.next(m.simple_lat_debug);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Debug_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roscpp_tutorials::Debug_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roscpp_tutorials::Debug_<ContainerAllocator>& v)
  {
    s << indent << "simple_lon_debug: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::SimpleLonDebug_<ContainerAllocator> >::stream(s, indent + "  ", v.simple_lon_debug);
    s << indent << "simple_lat_debug: ";
    s << std::endl;
    Printer< ::roscpp_tutorials::SimpleLatDebug_<ContainerAllocator> >::stream(s, indent + "  ", v.simple_lat_debug);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSCPP_TUTORIALS_MESSAGE_DEBUG_H