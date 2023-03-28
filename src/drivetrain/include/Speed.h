#ifndef _ROS_runt_rover_Speed_h
#define _ROS_runt_rover_Speed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace runt_rover
{

  class Speed : public ros::Msg
  {
    public:
      typedef int64_t _leftspeed_type;
      _leftspeed_type leftspeed;
      typedef int64_t _rightspeed_type;
      _rightspeed_type rightspeed;
      typedef int64_t _rightdirection_type;
      _rightdirection_type rightdirection;
      typedef int64_t _leftdirection_type;
      _leftdirection_type leftdirection;

    Speed():
      leftspeed(0),
      rightspeed(0),
      rightdirection(0),
      leftdirection(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_leftspeed;
      u_leftspeed.real = this->leftspeed;
      *(outbuffer + offset + 0) = (u_leftspeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftspeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftspeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftspeed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_leftspeed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_leftspeed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_leftspeed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_leftspeed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->leftspeed);
      union {
        int64_t real;
        uint64_t base;
      } u_rightspeed;
      u_rightspeed.real = this->rightspeed;
      *(outbuffer + offset + 0) = (u_rightspeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightspeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightspeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightspeed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rightspeed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rightspeed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rightspeed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rightspeed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rightspeed);
      union {
        int64_t real;
        uint64_t base;
      } u_rightdirection;
      u_rightdirection.real = this->rightdirection;
      *(outbuffer + offset + 0) = (u_rightdirection.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightdirection.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightdirection.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightdirection.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rightdirection.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rightdirection.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rightdirection.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rightdirection.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rightdirection);
      union {
        int64_t real;
        uint64_t base;
      } u_leftdirection;
      u_leftdirection.real = this->leftdirection;
      *(outbuffer + offset + 0) = (u_leftdirection.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftdirection.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftdirection.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftdirection.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_leftdirection.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_leftdirection.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_leftdirection.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_leftdirection.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->leftdirection);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_leftspeed;
      u_leftspeed.base = 0;
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_leftspeed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->leftspeed = u_leftspeed.real;
      offset += sizeof(this->leftspeed);
      union {
        int64_t real;
        uint64_t base;
      } u_rightspeed;
      u_rightspeed.base = 0;
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rightspeed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rightspeed = u_rightspeed.real;
      offset += sizeof(this->rightspeed);
      union {
        int64_t real;
        uint64_t base;
      } u_rightdirection;
      u_rightdirection.base = 0;
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rightdirection.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rightdirection = u_rightdirection.real;
      offset += sizeof(this->rightdirection);
      union {
        int64_t real;
        uint64_t base;
      } u_leftdirection;
      u_leftdirection.base = 0;
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_leftdirection.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->leftdirection = u_leftdirection.real;
      offset += sizeof(this->leftdirection);
     return offset;
    }

    virtual const char * getType() override { return "runt_rover/Speed"; };
    virtual const char * getMD5() override { return "e411503b1c3b7e55df6f17adf4515654"; };

  };

}
#endif
