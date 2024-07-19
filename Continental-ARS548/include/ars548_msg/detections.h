// Generated by gencpp from file ars548_msg/detections.msg
// DO NOT EDIT!


#ifndef ARS548_MSG_MESSAGE_DETECTIONS_H
#define ARS548_MSG_MESSAGE_DETECTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ars548_msg
{
template <class ContainerAllocator>
struct detections_
{
  typedef detections_<ContainerAllocator> Type;

  detections_()
    : header()
    , f_x(0.0)
    , f_y(0.0)
    , f_z(0.0)
    , u_InvalidFlags(0)
    , f_RangeRate(0.0)
    , f_RangeRateSTD(0.0)
    , s_RCS(0)
    , u_MeasurementID(0)
    , u_PositivePredictiveValue(0)
    , u_Classification(0)
    , u_MultiTargetProbability(0)
    , u_ObjectID(0)
    , u_AmbiguityFlag(0)
    , u_SortIndex(0)  {
    }
  detections_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , f_x(0.0)
    , f_y(0.0)
    , f_z(0.0)
    , u_InvalidFlags(0)
    , f_RangeRate(0.0)
    , f_RangeRateSTD(0.0)
    , s_RCS(0)
    , u_MeasurementID(0)
    , u_PositivePredictiveValue(0)
    , u_Classification(0)
    , u_MultiTargetProbability(0)
    , u_ObjectID(0)
    , u_AmbiguityFlag(0)
    , u_SortIndex(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _f_x_type;
  _f_x_type f_x;

   typedef float _f_y_type;
  _f_y_type f_y;

   typedef float _f_z_type;
  _f_z_type f_z;

   typedef uint8_t _u_InvalidFlags_type;
  _u_InvalidFlags_type u_InvalidFlags;

   typedef float _f_RangeRate_type;
  _f_RangeRate_type f_RangeRate;

   typedef float _f_RangeRateSTD_type;
  _f_RangeRateSTD_type f_RangeRateSTD;

   typedef int8_t _s_RCS_type;
  _s_RCS_type s_RCS;

   typedef uint16_t _u_MeasurementID_type;
  _u_MeasurementID_type u_MeasurementID;

   typedef uint8_t _u_PositivePredictiveValue_type;
  _u_PositivePredictiveValue_type u_PositivePredictiveValue;

   typedef uint8_t _u_Classification_type;
  _u_Classification_type u_Classification;

   typedef uint8_t _u_MultiTargetProbability_type;
  _u_MultiTargetProbability_type u_MultiTargetProbability;

   typedef uint16_t _u_ObjectID_type;
  _u_ObjectID_type u_ObjectID;

   typedef uint8_t _u_AmbiguityFlag_type;
  _u_AmbiguityFlag_type u_AmbiguityFlag;

   typedef uint16_t _u_SortIndex_type;
  _u_SortIndex_type u_SortIndex;





  typedef boost::shared_ptr< ::ars548_msg::detections_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ars548_msg::detections_<ContainerAllocator> const> ConstPtr;

}; // struct detections_

typedef ::ars548_msg::detections_<std::allocator<void> > detections;

typedef boost::shared_ptr< ::ars548_msg::detections > detectionsPtr;
typedef boost::shared_ptr< ::ars548_msg::detections const> detectionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ars548_msg::detections_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ars548_msg::detections_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ars548_msg::detections_<ContainerAllocator1> & lhs, const ::ars548_msg::detections_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.f_x == rhs.f_x &&
    lhs.f_y == rhs.f_y &&
    lhs.f_z == rhs.f_z &&
    lhs.u_InvalidFlags == rhs.u_InvalidFlags &&
    lhs.f_RangeRate == rhs.f_RangeRate &&
    lhs.f_RangeRateSTD == rhs.f_RangeRateSTD &&
    lhs.s_RCS == rhs.s_RCS &&
    lhs.u_MeasurementID == rhs.u_MeasurementID &&
    lhs.u_PositivePredictiveValue == rhs.u_PositivePredictiveValue &&
    lhs.u_Classification == rhs.u_Classification &&
    lhs.u_MultiTargetProbability == rhs.u_MultiTargetProbability &&
    lhs.u_ObjectID == rhs.u_ObjectID &&
    lhs.u_AmbiguityFlag == rhs.u_AmbiguityFlag &&
    lhs.u_SortIndex == rhs.u_SortIndex;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ars548_msg::detections_<ContainerAllocator1> & lhs, const ::ars548_msg::detections_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ars548_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ars548_msg::detections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ars548_msg::detections_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ars548_msg::detections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ars548_msg::detections_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ars548_msg::detections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ars548_msg::detections_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ars548_msg::detections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f5e661caf3e71fdf26903f146bda01f";
  }

  static const char* value(const ::ars548_msg::detections_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f5e661caf3e71fdULL;
  static const uint64_t static_value2 = 0xf26903f146bda01fULL;
};

template<class ContainerAllocator>
struct DataType< ::ars548_msg::detections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ars548_msg/detections";
  }

  static const char* value(const ::ars548_msg::detections_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ars548_msg::detections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"  \n"
"float32 f_x\n"
"float32 f_y\n"
"float32 f_z\n"
"uint8 u_InvalidFlags\n"
"float32 f_RangeRate\n"
"float32 f_RangeRateSTD\n"
"int8 s_RCS\n"
"uint16 u_MeasurementID\n"
"uint8 u_PositivePredictiveValue\n"
"uint8 u_Classification\n"
"uint8 u_MultiTargetProbability\n"
"uint16 u_ObjectID\n"
"uint8 u_AmbiguityFlag\n"
"uint16 u_SortIndex\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::ars548_msg::detections_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ars548_msg::detections_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.f_x);
      stream.next(m.f_y);
      stream.next(m.f_z);
      stream.next(m.u_InvalidFlags);
      stream.next(m.f_RangeRate);
      stream.next(m.f_RangeRateSTD);
      stream.next(m.s_RCS);
      stream.next(m.u_MeasurementID);
      stream.next(m.u_PositivePredictiveValue);
      stream.next(m.u_Classification);
      stream.next(m.u_MultiTargetProbability);
      stream.next(m.u_ObjectID);
      stream.next(m.u_AmbiguityFlag);
      stream.next(m.u_SortIndex);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct detections_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ars548_msg::detections_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ars548_msg::detections_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "f_x: ";
    Printer<float>::stream(s, indent + "  ", v.f_x);
    s << indent << "f_y: ";
    Printer<float>::stream(s, indent + "  ", v.f_y);
    s << indent << "f_z: ";
    Printer<float>::stream(s, indent + "  ", v.f_z);
    s << indent << "u_InvalidFlags: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.u_InvalidFlags);
    s << indent << "f_RangeRate: ";
    Printer<float>::stream(s, indent + "  ", v.f_RangeRate);
    s << indent << "f_RangeRateSTD: ";
    Printer<float>::stream(s, indent + "  ", v.f_RangeRateSTD);
    s << indent << "s_RCS: ";
    Printer<int8_t>::stream(s, indent + "  ", v.s_RCS);
    s << indent << "u_MeasurementID: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.u_MeasurementID);
    s << indent << "u_PositivePredictiveValue: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.u_PositivePredictiveValue);
    s << indent << "u_Classification: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.u_Classification);
    s << indent << "u_MultiTargetProbability: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.u_MultiTargetProbability);
    s << indent << "u_ObjectID: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.u_ObjectID);
    s << indent << "u_AmbiguityFlag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.u_AmbiguityFlag);
    s << indent << "u_SortIndex: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.u_SortIndex);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARS548_MSG_MESSAGE_DETECTIONS_H
