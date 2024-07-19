// Generated by gencpp from file ars548_msg/SteeringAngleFrontAxle.msg
// DO NOT EDIT!


#ifndef ARS548_MSG_MESSAGE_STEERINGANGLEFRONTAXLE_H
#define ARS548_MSG_MESSAGE_STEERINGANGLEFRONTAXLE_H


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
struct SteeringAngleFrontAxle_
{
  typedef SteeringAngleFrontAxle_<ContainerAllocator> Type;

  SteeringAngleFrontAxle_()
    : header()
    , QualifierSteeringAngleFrontAxle(0)
    , SteeringAngleFrontAxleErrAmp(0.0)
    , SteeringAngleFrontAxleErrAmp_InvalidFlag(0)
    , SteeringAngleFrontAxle(0.0)
    , SteeringAngleFrontAxle_InvalidFlag(0)
    , SteeringAngleFrontAxleEventDataQualifier(0)  {
    }
  SteeringAngleFrontAxle_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , QualifierSteeringAngleFrontAxle(0)
    , SteeringAngleFrontAxleErrAmp(0.0)
    , SteeringAngleFrontAxleErrAmp_InvalidFlag(0)
    , SteeringAngleFrontAxle(0.0)
    , SteeringAngleFrontAxle_InvalidFlag(0)
    , SteeringAngleFrontAxleEventDataQualifier(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _QualifierSteeringAngleFrontAxle_type;
  _QualifierSteeringAngleFrontAxle_type QualifierSteeringAngleFrontAxle;

   typedef float _SteeringAngleFrontAxleErrAmp_type;
  _SteeringAngleFrontAxleErrAmp_type SteeringAngleFrontAxleErrAmp;

   typedef uint8_t _SteeringAngleFrontAxleErrAmp_InvalidFlag_type;
  _SteeringAngleFrontAxleErrAmp_InvalidFlag_type SteeringAngleFrontAxleErrAmp_InvalidFlag;

   typedef float _SteeringAngleFrontAxle_type;
  _SteeringAngleFrontAxle_type SteeringAngleFrontAxle;

   typedef uint8_t _SteeringAngleFrontAxle_InvalidFlag_type;
  _SteeringAngleFrontAxle_InvalidFlag_type SteeringAngleFrontAxle_InvalidFlag;

   typedef uint8_t _SteeringAngleFrontAxleEventDataQualifier_type;
  _SteeringAngleFrontAxleEventDataQualifier_type SteeringAngleFrontAxleEventDataQualifier;





  typedef boost::shared_ptr< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> const> ConstPtr;

}; // struct SteeringAngleFrontAxle_

typedef ::ars548_msg::SteeringAngleFrontAxle_<std::allocator<void> > SteeringAngleFrontAxle;

typedef boost::shared_ptr< ::ars548_msg::SteeringAngleFrontAxle > SteeringAngleFrontAxlePtr;
typedef boost::shared_ptr< ::ars548_msg::SteeringAngleFrontAxle const> SteeringAngleFrontAxleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator1> & lhs, const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.QualifierSteeringAngleFrontAxle == rhs.QualifierSteeringAngleFrontAxle &&
    lhs.SteeringAngleFrontAxleErrAmp == rhs.SteeringAngleFrontAxleErrAmp &&
    lhs.SteeringAngleFrontAxleErrAmp_InvalidFlag == rhs.SteeringAngleFrontAxleErrAmp_InvalidFlag &&
    lhs.SteeringAngleFrontAxle == rhs.SteeringAngleFrontAxle &&
    lhs.SteeringAngleFrontAxle_InvalidFlag == rhs.SteeringAngleFrontAxle_InvalidFlag &&
    lhs.SteeringAngleFrontAxleEventDataQualifier == rhs.SteeringAngleFrontAxleEventDataQualifier;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator1> & lhs, const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ars548_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "99d1eb99bbf871691cf14ff247e0b6c9";
  }

  static const char* value(const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x99d1eb99bbf87169ULL;
  static const uint64_t static_value2 = 0x1cf14ff247e0b6c9ULL;
};

template<class ContainerAllocator>
struct DataType< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ars548_msg/SteeringAngleFrontAxle";
  }

  static const char* value(const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"uint8 QualifierSteeringAngleFrontAxle\n"
"float32 SteeringAngleFrontAxleErrAmp\n"
"uint8 SteeringAngleFrontAxleErrAmp_InvalidFlag\n"
"float32 SteeringAngleFrontAxle\n"
"uint8 SteeringAngleFrontAxle_InvalidFlag\n"
"uint8 SteeringAngleFrontAxleEventDataQualifier\n"
"\n"
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

  static const char* value(const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.QualifierSteeringAngleFrontAxle);
      stream.next(m.SteeringAngleFrontAxleErrAmp);
      stream.next(m.SteeringAngleFrontAxleErrAmp_InvalidFlag);
      stream.next(m.SteeringAngleFrontAxle);
      stream.next(m.SteeringAngleFrontAxle_InvalidFlag);
      stream.next(m.SteeringAngleFrontAxleEventDataQualifier);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SteeringAngleFrontAxle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ars548_msg::SteeringAngleFrontAxle_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "QualifierSteeringAngleFrontAxle: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.QualifierSteeringAngleFrontAxle);
    s << indent << "SteeringAngleFrontAxleErrAmp: ";
    Printer<float>::stream(s, indent + "  ", v.SteeringAngleFrontAxleErrAmp);
    s << indent << "SteeringAngleFrontAxleErrAmp_InvalidFlag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.SteeringAngleFrontAxleErrAmp_InvalidFlag);
    s << indent << "SteeringAngleFrontAxle: ";
    Printer<float>::stream(s, indent + "  ", v.SteeringAngleFrontAxle);
    s << indent << "SteeringAngleFrontAxle_InvalidFlag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.SteeringAngleFrontAxle_InvalidFlag);
    s << indent << "SteeringAngleFrontAxleEventDataQualifier: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.SteeringAngleFrontAxleEventDataQualifier);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARS548_MSG_MESSAGE_STEERINGANGLEFRONTAXLE_H