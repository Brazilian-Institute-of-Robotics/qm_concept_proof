#ifndef _ROS_SERVICE_ModString_h
#define _ROS_SERVICE_ModString_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace beginner_tutorials
{

static const char MODSTRING[] = "beginner_tutorials/ModString";

  class ModStringRequest : public ros::Msg
  {
    public:
      const char* s;

    ModStringRequest():
      s("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_s = strlen(this->s);
      memcpy(outbuffer + offset, &length_s, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->s, length_s);
      offset += length_s;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_s;
      memcpy(&length_s, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_s; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_s-1]=0;
      this->s = (char *)(inbuffer + offset-1);
      offset += length_s;
     return offset;
    }

    const char * getType(){ return MODSTRING; };
    const char * getMD5(){ return "81af3411577d82a6786258523fc891ce"; };

  };

  class ModStringResponse : public ros::Msg
  {
    public:

    ModStringResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return MODSTRING; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ModString {
    public:
    typedef ModStringRequest Request;
    typedef ModStringResponse Response;
  };

}
#endif
