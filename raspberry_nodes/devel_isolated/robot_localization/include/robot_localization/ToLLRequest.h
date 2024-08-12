// Generated by gencpp from file robot_localization/ToLLRequest.msg
// DO NOT EDIT!


#ifndef ROBOT_LOCALIZATION_MESSAGE_TOLLREQUEST_H
#define ROBOT_LOCALIZATION_MESSAGE_TOLLREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace robot_localization
{
template <class ContainerAllocator>
struct ToLLRequest_
{
  typedef ToLLRequest_<ContainerAllocator> Type;

  ToLLRequest_()
    : map_point()  {
    }
  ToLLRequest_(const ContainerAllocator& _alloc)
    : map_point(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _map_point_type;
  _map_point_type map_point;





  typedef boost::shared_ptr< ::robot_localization::ToLLRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_localization::ToLLRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ToLLRequest_

typedef ::robot_localization::ToLLRequest_<std::allocator<void> > ToLLRequest;

typedef boost::shared_ptr< ::robot_localization::ToLLRequest > ToLLRequestPtr;
typedef boost::shared_ptr< ::robot_localization::ToLLRequest const> ToLLRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_localization::ToLLRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_localization::ToLLRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_localization::ToLLRequest_<ContainerAllocator1> & lhs, const ::robot_localization::ToLLRequest_<ContainerAllocator2> & rhs)
{
  return lhs.map_point == rhs.map_point;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_localization::ToLLRequest_<ContainerAllocator1> & lhs, const ::robot_localization::ToLLRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_localization

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robot_localization::ToLLRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_localization::ToLLRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_localization::ToLLRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_localization::ToLLRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_localization::ToLLRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_localization::ToLLRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_localization::ToLLRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "326fc0ec1385c52a253c06e024d9f49e";
  }

  static const char* value(const ::robot_localization::ToLLRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x326fc0ec1385c52aULL;
  static const uint64_t static_value2 = 0x253c06e024d9f49eULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_localization::ToLLRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_localization/ToLLRequest";
  }

  static const char* value(const ::robot_localization::ToLLRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_localization::ToLLRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point map_point\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::robot_localization::ToLLRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_localization::ToLLRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.map_point);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToLLRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_localization::ToLLRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_localization::ToLLRequest_<ContainerAllocator>& v)
  {
    s << indent << "map_point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.map_point);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_LOCALIZATION_MESSAGE_TOLLREQUEST_H
