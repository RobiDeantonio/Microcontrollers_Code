#ifndef _ROS_ceres_CeresArduinoConfig_h
#define _ROS_ceres_CeresArduinoConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ceres
{

  class CeresArduinoConfig : public ros::Msg
  {
    public:
      typedef uint16_t _K1_type;
      _K1_type K1;
      typedef uint16_t _K2_type;
      _K2_type K2;
      typedef uint16_t _freq_type;
      _freq_type freq;
      typedef uint16_t _Kdl_type;
      _Kdl_type Kdl;
      typedef uint16_t _Kdr_type;
      _Kdr_type Kdr;

    CeresArduinoConfig():
      K1(0),
      K2(0),
      freq(0),
      Kdl(0),
      Kdr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->K1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->K1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->K1);
      *(outbuffer + offset + 0) = (this->K2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->K2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->K2);
      *(outbuffer + offset + 0) = (this->freq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->freq >> (8 * 1)) & 0xFF;
      offset += sizeof(this->freq);
      *(outbuffer + offset + 0) = (this->Kdl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Kdl >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Kdl);
      *(outbuffer + offset + 0) = (this->Kdr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Kdr >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Kdr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->K1 =  ((uint16_t) (*(inbuffer + offset)));
      this->K1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->K1);
      this->K2 =  ((uint16_t) (*(inbuffer + offset)));
      this->K2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->K2);
      this->freq =  ((uint16_t) (*(inbuffer + offset)));
      this->freq |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->freq);
      this->Kdl =  ((uint16_t) (*(inbuffer + offset)));
      this->Kdl |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Kdl);
      this->Kdr =  ((uint16_t) (*(inbuffer + offset)));
      this->Kdr |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Kdr);
     return offset;
    }

    const char * getType(){ return "ceres/CeresArduinoConfig"; };
    const char * getMD5(){ return "49273e6f090cea543f145cb9d8868290"; };

  };

}
#endif