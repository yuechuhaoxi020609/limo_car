// Generated by gencpp from file astra_camera/GetCameraInfo.msg
// DO NOT EDIT!


#ifndef ASTRA_CAMERA_MESSAGE_GETCAMERAINFO_H
#define ASTRA_CAMERA_MESSAGE_GETCAMERAINFO_H

#include <ros/service_traits.h>


#include <astra_camera/GetCameraInfoRequest.h>
#include <astra_camera/GetCameraInfoResponse.h>


namespace astra_camera
{

struct GetCameraInfo
{

typedef GetCameraInfoRequest Request;
typedef GetCameraInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetCameraInfo
} // namespace astra_camera


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::astra_camera::GetCameraInfo > {
  static const char* value()
  {
    return "aacf7dfed1a501be45f34981291a5579";
  }

  static const char* value(const ::astra_camera::GetCameraInfo&) { return value(); }
};

template<>
struct DataType< ::astra_camera::GetCameraInfo > {
  static const char* value()
  {
    return "astra_camera/GetCameraInfo";
  }

  static const char* value(const ::astra_camera::GetCameraInfo&) { return value(); }
};


// service_traits::MD5Sum< ::astra_camera::GetCameraInfoRequest> should match
// service_traits::MD5Sum< ::astra_camera::GetCameraInfo >
template<>
struct MD5Sum< ::astra_camera::GetCameraInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::astra_camera::GetCameraInfo >::value();
  }
  static const char* value(const ::astra_camera::GetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::astra_camera::GetCameraInfoRequest> should match
// service_traits::DataType< ::astra_camera::GetCameraInfo >
template<>
struct DataType< ::astra_camera::GetCameraInfoRequest>
{
  static const char* value()
  {
    return DataType< ::astra_camera::GetCameraInfo >::value();
  }
  static const char* value(const ::astra_camera::GetCameraInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::astra_camera::GetCameraInfoResponse> should match
// service_traits::MD5Sum< ::astra_camera::GetCameraInfo >
template<>
struct MD5Sum< ::astra_camera::GetCameraInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::astra_camera::GetCameraInfo >::value();
  }
  static const char* value(const ::astra_camera::GetCameraInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::astra_camera::GetCameraInfoResponse> should match
// service_traits::DataType< ::astra_camera::GetCameraInfo >
template<>
struct DataType< ::astra_camera::GetCameraInfoResponse>
{
  static const char* value()
  {
    return DataType< ::astra_camera::GetCameraInfo >::value();
  }
  static const char* value(const ::astra_camera::GetCameraInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ASTRA_CAMERA_MESSAGE_GETCAMERAINFO_H
