#ifndef _ROS_ceres_ArduinoLog_h
#define _ROS_ceres_ArduinoLog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ceres
{

  class ArduinoLog : public ros::Msg
  {
    public:
      typedef float _vx_type;
      _vx_type vx;
      typedef float _wz_type;
      _wz_type wz;
      typedef float _vl_type;
      _vl_type vl;
      typedef float _vr_type;
      _vr_type vr;
      typedef int8_t _AU_type;
      _AU_type AU;
      typedef int8_t _AUX_type;
      _AUX_type AUX;

    ArduinoLog():
      vx(0),
      wz(0),
      vl(0),
      vr(0),
      AU(0),
      AUX(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->wz);
      offset += serializeAvrFloat64(outbuffer + offset, this->vl);
      offset += serializeAvrFloat64(outbuffer + offset, this->vr);
      union {
        int8_t real;
        uint8_t base;
      } u_AU;
      u_AU.real = this->AU;
      *(outbuffer + offset + 0) = (u_AU.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->AU);
      union {
        int8_t real;
        uint8_t base;
      } u_AUX;
      u_AUX.real = this->AUX;
      *(outbuffer + offset + 0) = (u_AUX.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->AUX);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->wz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vl));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vr));
      union {
        int8_t real;
        uint8_t base;
      } u_AU;
      u_AU.base = 0;
      u_AU.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->AU = u_AU.real;
      offset += sizeof(this->AU);
      union {
        int8_t real;
        uint8_t base;
      } u_AUX;
      u_AUX.base = 0;
      u_AUX.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->AUX = u_AUX.real;
      offset += sizeof(this->AUX);
     return offset;
    }

    const char * getType(){ return "ceres/ArduinoLog"; };
    const char * getMD5(){ return "7d009c452da6c74a783c822714717799"; };

  };

}
#endif