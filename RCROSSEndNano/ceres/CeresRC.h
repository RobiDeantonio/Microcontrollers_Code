#ifndef _ROS_ceres_CeresRC_h
#define _ROS_ceres_CeresRC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ceres
{

  class CeresRC : public ros::Msg
  {
    public:
      typedef int16_t _CH1_type;
      _CH1_type CH1;
      typedef int16_t _CH2_type;
      _CH2_type CH2;
      typedef int16_t _CH3_type;
      _CH3_type CH3;
      typedef int16_t _CH4_type;
      _CH4_type CH4;
      typedef int8_t _emergency_type;
      _emergency_type emergency;
      typedef int8_t _AUX_type;
      _AUX_type AUX;

    CeresRC():
      CH1(0),
      CH2(0),
      CH3(0),
      CH4(0),
      emergency(0),
      AUX(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_CH1;
      u_CH1.real = this->CH1;
      *(outbuffer + offset + 0) = (u_CH1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_CH1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->CH1);
      union {
        int16_t real;
        uint16_t base;
      } u_CH2;
      u_CH2.real = this->CH2;
      *(outbuffer + offset + 0) = (u_CH2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_CH2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->CH2);
      union {
        int16_t real;
        uint16_t base;
      } u_CH3;
      u_CH3.real = this->CH3;
      *(outbuffer + offset + 0) = (u_CH3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_CH3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->CH3);
      union {
        int16_t real;
        uint16_t base;
      } u_CH4;
      u_CH4.real = this->CH4;
      *(outbuffer + offset + 0) = (u_CH4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_CH4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->CH4);
      union {
        int8_t real;
        uint8_t base;
      } u_emergency;
      u_emergency.real = this->emergency;
      *(outbuffer + offset + 0) = (u_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency);
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
      union {
        int16_t real;
        uint16_t base;
      } u_CH1;
      u_CH1.base = 0;
      u_CH1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_CH1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->CH1 = u_CH1.real;
      offset += sizeof(this->CH1);
      union {
        int16_t real;
        uint16_t base;
      } u_CH2;
      u_CH2.base = 0;
      u_CH2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_CH2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->CH2 = u_CH2.real;
      offset += sizeof(this->CH2);
      union {
        int16_t real;
        uint16_t base;
      } u_CH3;
      u_CH3.base = 0;
      u_CH3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_CH3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->CH3 = u_CH3.real;
      offset += sizeof(this->CH3);
      union {
        int16_t real;
        uint16_t base;
      } u_CH4;
      u_CH4.base = 0;
      u_CH4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_CH4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->CH4 = u_CH4.real;
      offset += sizeof(this->CH4);
      union {
        int8_t real;
        uint8_t base;
      } u_emergency;
      u_emergency.base = 0;
      u_emergency.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency = u_emergency.real;
      offset += sizeof(this->emergency);
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

    const char * getType(){ return "ceres/CeresRC"; };
    const char * getMD5(){ return "0b3cca7b28c64076b9b944214706bb93"; };

  };

}
#endif