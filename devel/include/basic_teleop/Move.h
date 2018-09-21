// Generated by gencpp from file basic_teleop/Move.msg
// DO NOT EDIT!


#ifndef BASIC_TELEOP_MESSAGE_MOVE_H
#define BASIC_TELEOP_MESSAGE_MOVE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace basic_teleop
{
template <class ContainerAllocator>
struct Move_
{
  typedef Move_<ContainerAllocator> Type;

  Move_()
    : direction()
    , duration(0)  {
    }
  Move_(const ContainerAllocator& _alloc)
    : direction(_alloc)
    , duration(0)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _direction_type;
  _direction_type direction;

   typedef int16_t _duration_type;
  _duration_type duration;




  typedef boost::shared_ptr< ::basic_teleop::Move_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::basic_teleop::Move_<ContainerAllocator> const> ConstPtr;

}; // struct Move_

typedef ::basic_teleop::Move_<std::allocator<void> > Move;

typedef boost::shared_ptr< ::basic_teleop::Move > MovePtr;
typedef boost::shared_ptr< ::basic_teleop::Move const> MoveConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::basic_teleop::Move_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::basic_teleop::Move_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace basic_teleop

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'basic_teleop': ['/home/will/catkin_ws/src/basic_teleop/msg'], 'sensor_msgs': ['/opt/ros/jade/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/jade/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/jade/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::basic_teleop::Move_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::basic_teleop::Move_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::basic_teleop::Move_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::basic_teleop::Move_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::basic_teleop::Move_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::basic_teleop::Move_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::basic_teleop::Move_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39ad474dc541f591e27d6995c7b85b53";
  }

  static const char* value(const ::basic_teleop::Move_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39ad474dc541f591ULL;
  static const uint64_t static_value2 = 0xe27d6995c7b85b53ULL;
};

template<class ContainerAllocator>
struct DataType< ::basic_teleop::Move_<ContainerAllocator> >
{
  static const char* value()
  {
    return "basic_teleop/Move";
  }

  static const char* value(const ::basic_teleop::Move_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::basic_teleop::Move_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string direction\n\
int16 duration\n\
";
  }

  static const char* value(const ::basic_teleop::Move_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::basic_teleop::Move_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.direction);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Move_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::basic_teleop::Move_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::basic_teleop::Move_<ContainerAllocator>& v)
  {
    s << indent << "direction: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.direction);
    s << indent << "duration: ";
    Printer<int16_t>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BASIC_TELEOP_MESSAGE_MOVE_H