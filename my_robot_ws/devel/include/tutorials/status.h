// Generated by gencpp from file tutorials/status.msg
// DO NOT EDIT!


#ifndef TUTORIALS_MESSAGE_STATUS_H
#define TUTORIALS_MESSAGE_STATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tutorials
{
template <class ContainerAllocator>
struct status_
{
  typedef status_<ContainerAllocator> Type;

  status_()
    : message()
    , x(0.0)
    , y(0.0)  {
    }
  status_(const ContainerAllocator& _alloc)
    : message(_alloc)
    , x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::tutorials::status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tutorials::status_<ContainerAllocator> const> ConstPtr;

}; // struct status_

typedef ::tutorials::status_<std::allocator<void> > status;

typedef boost::shared_ptr< ::tutorials::status > statusPtr;
typedef boost::shared_ptr< ::tutorials::status const> statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tutorials::status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tutorials::status_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tutorials::status_<ContainerAllocator1> & lhs, const ::tutorials::status_<ContainerAllocator2> & rhs)
{
  return lhs.message == rhs.message &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tutorials::status_<ContainerAllocator1> & lhs, const ::tutorials::status_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tutorials

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tutorials::status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tutorials::status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tutorials::status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tutorials::status_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tutorials::status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tutorials::status_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tutorials::status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7e1f88ae0dc8c54957ce388602e32426";
  }

  static const char* value(const ::tutorials::status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7e1f88ae0dc8c549ULL;
  static const uint64_t static_value2 = 0x57ce388602e32426ULL;
};

template<class ContainerAllocator>
struct DataType< ::tutorials::status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tutorials/status";
  }

  static const char* value(const ::tutorials::status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tutorials::status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string message\n"
"float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::tutorials::status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tutorials::status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tutorials::status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tutorials::status_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TUTORIALS_MESSAGE_STATUS_H