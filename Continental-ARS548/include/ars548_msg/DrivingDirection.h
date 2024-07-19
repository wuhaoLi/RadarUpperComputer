// Generated by gencpp from file ars548_msg/DrivingDirection.msg
// DO NOT EDIT!


#ifndef ARS548_MSG_MESSAGE_DRIVINGDIRECTION_H
#define ARS548_MSG_MESSAGE_DRIVINGDIRECTION_H


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
struct DrivingDirection_
{
  typedef DrivingDirection_<ContainerAllocator> Type;

  DrivingDirection_()
    : header()
    , DrivingDirectionUnconfirmed(0)
    , DrivingDirectionConfirmed(0)  {
    }
  DrivingDirection_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , DrivingDirectionUnconfirmed(0)
    , DrivingDirectionConfirmed(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _DrivingDirectionUnconfirmed_type;
  _DrivingDirectionUnconfirmed_type DrivingDirectionUnconfirmed;

   typedef uint8_t _DrivingDirectionConfirmed_type;
  _DrivingDirectionConfirmed_type DrivingDirectionConfirmed;





  typedef boost::shared_ptr< ::ars548_msg::DrivingDirection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ars548_msg::DrivingDirection_<ContainerAllocator> const> ConstPtr;

}; // struct DrivingDirection_

typedef ::ars548_msg::DrivingDirection_<std::allocator<void> > DrivingDirection;

typedef boost::shared_ptr< ::ars548_msg::DrivingDirection > DrivingDirectionPtr;
typedef boost::shared_ptr< ::ars548_msg::DrivingDirection const> DrivingDirectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ars548_msg::DrivingDirection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ars548_msg::DrivingDirection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ars548_msg::DrivingDirection_<ContainerAllocator1> & lhs, const ::ars548_msg::DrivingDirection_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.DrivingDirectionUnconfirmed == rhs.DrivingDirectionUnconfirmed &&
    lhs.DrivingDirectionConfirmed == rhs.DrivingDirectionConfirmed;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ars548_msg::DrivingDirection_<ContainerAllocator1> & lhs, const ::ars548_msg::DrivingDirection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ars548_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ars548_msg::DrivingDirection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ars548_msg::DrivingDirection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ars548_msg::DrivingDirection_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c828bb62f02d5b856f7cfebbb61f4dc";
  }

  static const char* value(const ::ars548_msg::DrivingDirection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c828bb62f02d5b8ULL;
  static const uint64_t static_value2 = 0x56f7cfebbb61f4dcULL;
};

template<class ContainerAllocator>
struct DataType< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ars548_msg/DrivingDirection";
  }

  static const char* value(const ::ars548_msg::DrivingDirection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"uint8 DrivingDirectionUnconfirmed\n"
"uint8 DrivingDirectionConfirmed\n"
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

  static const char* value(const ::ars548_msg::DrivingDirection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.DrivingDirectionUnconfirmed);
      stream.next(m.DrivingDirectionConfirmed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DrivingDirection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ars548_msg::DrivingDirection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ars548_msg::DrivingDirection_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "DrivingDirectionUnconfirmed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.DrivingDirectionUnconfirmed);
    s << indent << "DrivingDirectionConfirmed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.DrivingDirectionConfirmed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARS548_MSG_MESSAGE_DRIVINGDIRECTION_H
