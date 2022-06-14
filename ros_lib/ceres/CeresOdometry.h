#ifndef _ROS_ceres_CeresOdometry_h
#define _ROS_ceres_CeresOdometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ceres
{

  class CeresOdometry : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _rz_type;
      _rz_type rz;
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _wz_type;
      _wz_type wz;

    CeresOdometry():
      x(0),
      y(0),
      rz(0),
      vx(0),
      vy(0),
      wz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->rz);
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vy);
      offset += serializeAvrFloat64(outbuffer + offset, this->wz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->wz));
     return offset;
    }

    const char * getType(){ return "ceres/CeresOdometry"; };
    const char * getMD5(){ return "d2e72c4711477d6822b8ef2dcdd814c6"; };

  };

}
#endif