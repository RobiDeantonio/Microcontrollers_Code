#ifndef _ROS_ceres_CeresArduinoLogging_h
#define _ROS_ceres_CeresArduinoLogging_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ceres
{

  class CeresArduinoLogging : public ros::Msg
  {
    public:
      typedef float _Vl_type;
      _Vl_type Vl;
      typedef float _Vr_type;
      _Vr_type Vr;
      typedef float _Ul_type;
      _Ul_type Ul;
      typedef float _Ur_type;
      _Ur_type Ur;

    CeresArduinoLogging():
      Vl(0),
      Vr(0),
      Ul(0),
      Ur(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->Vl);
      offset += serializeAvrFloat64(outbuffer + offset, this->Vr);
      offset += serializeAvrFloat64(outbuffer + offset, this->Ul);
      offset += serializeAvrFloat64(outbuffer + offset, this->Ur);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Vl));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Vr));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Ul));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Ur));
     return offset;
    }

    const char * getType(){ return "ceres/CeresArduinoLogging"; };
    const char * getMD5(){ return "ce6d8f05699a8b1cf6aaaa3f23727b11"; };

  };

}
#endif