/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/paul/ros_ws/src/brio_vision/msg/Data_Type.msg
 *
 */


#ifndef BRIO_VISION_MESSAGE_DATA_TYPE_H
#define BRIO_VISION_MESSAGE_DATA_TYPE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace brio_vision
{
template <class ContainerAllocator>
struct Data_Type_
{
  typedef Data_Type_<ContainerAllocator> Type;

  Data_Type_()
    : center_index(0)
    , head_conn_index(0)
    , back_conn_index(0)
    , piece_type()  {
    }
  Data_Type_(const ContainerAllocator& _alloc)
    : center_index(0)
    , head_conn_index(0)
    , back_conn_index(0)
    , piece_type(_alloc)  {
    }



   typedef int32_t _center_index_type;
  _center_index_type center_index;

   typedef int32_t _head_conn_index_type;
  _head_conn_index_type head_conn_index;

   typedef int32_t _back_conn_index_type;
  _back_conn_index_type back_conn_index;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _piece_type_type;
  _piece_type_type piece_type;




  typedef boost::shared_ptr< ::brio_vision::Data_Type_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brio_vision::Data_Type_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct Data_Type_

typedef ::brio_vision::Data_Type_<std::allocator<void> > Data_Type;

typedef boost::shared_ptr< ::brio_vision::Data_Type > Data_TypePtr;
typedef boost::shared_ptr< ::brio_vision::Data_Type const> Data_TypeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brio_vision::Data_Type_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brio_vision::Data_Type_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace brio_vision

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'brio_vision': ['/home/paul/ros_ws/src/brio_vision/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::brio_vision::Data_Type_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brio_vision::Data_Type_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brio_vision::Data_Type_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brio_vision::Data_Type_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brio_vision::Data_Type_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brio_vision::Data_Type_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brio_vision::Data_Type_<ContainerAllocator> >
{
  static const char* value()
  {
    return "56f44d48ac24562d5f582b0744aa0e09";
  }

  static const char* value(const ::brio_vision::Data_Type_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x56f44d48ac24562dULL;
  static const uint64_t static_value2 = 0x5f582b0744aa0e09ULL;
};

template<class ContainerAllocator>
struct DataType< ::brio_vision::Data_Type_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brio_vision/Data_Type";
  }

  static const char* value(const ::brio_vision::Data_Type_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brio_vision::Data_Type_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 center_index\n\
int32 head_conn_index\n\
int32 back_conn_index\n\
string piece_type\n\
";
  }

  static const char* value(const ::brio_vision::Data_Type_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brio_vision::Data_Type_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.center_index);
      stream.next(m.head_conn_index);
      stream.next(m.back_conn_index);
      stream.next(m.piece_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Data_Type_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brio_vision::Data_Type_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brio_vision::Data_Type_<ContainerAllocator>& v)
  {
    s << indent << "center_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.center_index);
    s << indent << "head_conn_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.head_conn_index);
    s << indent << "back_conn_index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.back_conn_index);
    s << indent << "piece_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.piece_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRIO_VISION_MESSAGE_DATA_TYPE_H