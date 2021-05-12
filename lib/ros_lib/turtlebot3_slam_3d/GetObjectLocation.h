#ifndef _ROS_SERVICE_GetObjectLocation_h
#define _ROS_SERVICE_GetObjectLocation_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseArray.h"

namespace turtlebot3_slam_3d
{

static const char GETOBJECTLOCATION[] = "turtlebot3_slam_3d/GetObjectLocation";

  class GetObjectLocationRequest : public ros::Msg
  {
    public:
      typedef const char* _object_name_type;
      _object_name_type object_name;

    GetObjectLocationRequest():
      object_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_object_name = strlen(this->object_name);
      varToArr(outbuffer + offset, length_object_name);
      offset += 4;
      memcpy(outbuffer + offset, this->object_name, length_object_name);
      offset += length_object_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_object_name;
      arrToVar(length_object_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_object_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_object_name-1]=0;
      this->object_name = (char *)(inbuffer + offset-1);
      offset += length_object_name;
     return offset;
    }

    const char * getType(){ return GETOBJECTLOCATION; };
    const char * getMD5(){ return "2f12226348d323c2e5b2031b3da53f1b"; };

  };

  class GetObjectLocationResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseArray _location_type;
      _location_type location;

    GetObjectLocationResponse():
      location()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->location.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->location.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETOBJECTLOCATION; };
    const char * getMD5(){ return "34cf2a1ed77cb3529e2b342e07fb56b1"; };

  };

  class GetObjectLocation {
    public:
    typedef GetObjectLocationRequest Request;
    typedef GetObjectLocationResponse Response;
  };

}
#endif
