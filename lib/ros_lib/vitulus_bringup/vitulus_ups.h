#ifndef _ROS_vitulus_bringup_vitulus_ups_h
#define _ROS_vitulus_bringup_vitulus_ups_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace vitulus_bringup
{

  class vitulus_ups : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _driver_type;
      _driver_type driver;
      typedef const char* _ups_status_type;
      _ups_status_type ups_status;
      typedef const char* _battery_status_type;
      _battery_status_type battery_status;
      typedef int16_t _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef int16_t _battery_current_type;
      _battery_current_type battery_current;
      typedef int16_t _battery_capacity_type;
      _battery_capacity_type battery_capacity;
      typedef uint8_t _battery_warning_type;
      _battery_warning_type battery_warning;
      typedef uint8_t _battery_low_type;
      _battery_low_type battery_low;
      typedef int16_t _input_voltage_type;
      _input_voltage_type input_voltage;
      typedef int16_t _input_current_type;
      _input_current_type input_current;
      typedef int16_t _output_voltage_type;
      _output_voltage_type output_voltage;
      typedef int16_t _output_current_type;
      _output_current_type output_current;
      typedef int16_t _cell1_voltage_type;
      _cell1_voltage_type cell1_voltage;
      typedef int16_t _cell2_voltage_type;
      _cell2_voltage_type cell2_voltage;
      typedef int16_t _cell3_voltage_type;
      _cell3_voltage_type cell3_voltage;
      typedef int16_t _cell4_voltage_type;
      _cell4_voltage_type cell4_voltage;
      typedef int16_t _cell5_voltage_type;
      _cell5_voltage_type cell5_voltage;
      typedef int16_t _cell6_voltage_type;
      _cell6_voltage_type cell6_voltage;

    vitulus_ups():
      header(),
      driver(""),
      ups_status(""),
      battery_status(""),
      battery_voltage(0),
      battery_current(0),
      battery_capacity(0),
      battery_warning(0),
      battery_low(0),
      input_voltage(0),
      input_current(0),
      output_voltage(0),
      output_current(0),
      cell1_voltage(0),
      cell2_voltage(0),
      cell3_voltage(0),
      cell4_voltage(0),
      cell5_voltage(0),
      cell6_voltage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_driver = strlen(this->driver);
      varToArr(outbuffer + offset, length_driver);
      offset += 4;
      memcpy(outbuffer + offset, this->driver, length_driver);
      offset += length_driver;
      uint32_t length_ups_status = strlen(this->ups_status);
      varToArr(outbuffer + offset, length_ups_status);
      offset += 4;
      memcpy(outbuffer + offset, this->ups_status, length_ups_status);
      offset += length_ups_status;
      uint32_t length_battery_status = strlen(this->battery_status);
      varToArr(outbuffer + offset, length_battery_status);
      offset += 4;
      memcpy(outbuffer + offset, this->battery_status, length_battery_status);
      offset += length_battery_status;
      union {
        int16_t real;
        uint16_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_battery_current;
      u_battery_current.real = this->battery_current;
      *(outbuffer + offset + 0) = (u_battery_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_current.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->battery_current);
      union {
        int16_t real;
        uint16_t base;
      } u_battery_capacity;
      u_battery_capacity.real = this->battery_capacity;
      *(outbuffer + offset + 0) = (u_battery_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_capacity.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->battery_capacity);
      *(outbuffer + offset + 0) = (this->battery_warning >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_warning);
      *(outbuffer + offset + 0) = (this->battery_low >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_low);
      union {
        int16_t real;
        uint16_t base;
      } u_input_voltage;
      u_input_voltage.real = this->input_voltage;
      *(outbuffer + offset + 0) = (u_input_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->input_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_input_current;
      u_input_current.real = this->input_current;
      *(outbuffer + offset + 0) = (u_input_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_current.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->input_current);
      union {
        int16_t real;
        uint16_t base;
      } u_output_voltage;
      u_output_voltage.real = this->output_voltage;
      *(outbuffer + offset + 0) = (u_output_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->output_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_output_current;
      u_output_current.real = this->output_current;
      *(outbuffer + offset + 0) = (u_output_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_current.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->output_current);
      union {
        int16_t real;
        uint16_t base;
      } u_cell1_voltage;
      u_cell1_voltage.real = this->cell1_voltage;
      *(outbuffer + offset + 0) = (u_cell1_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell1_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cell1_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell2_voltage;
      u_cell2_voltage.real = this->cell2_voltage;
      *(outbuffer + offset + 0) = (u_cell2_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell2_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cell2_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell3_voltage;
      u_cell3_voltage.real = this->cell3_voltage;
      *(outbuffer + offset + 0) = (u_cell3_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell3_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cell3_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell4_voltage;
      u_cell4_voltage.real = this->cell4_voltage;
      *(outbuffer + offset + 0) = (u_cell4_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell4_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cell4_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell5_voltage;
      u_cell5_voltage.real = this->cell5_voltage;
      *(outbuffer + offset + 0) = (u_cell5_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell5_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cell5_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell6_voltage;
      u_cell6_voltage.real = this->cell6_voltage;
      *(outbuffer + offset + 0) = (u_cell6_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell6_voltage.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cell6_voltage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_driver;
      arrToVar(length_driver, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_driver; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_driver-1]=0;
      this->driver = (char *)(inbuffer + offset-1);
      offset += length_driver;
      uint32_t length_ups_status;
      arrToVar(length_ups_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ups_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ups_status-1]=0;
      this->ups_status = (char *)(inbuffer + offset-1);
      offset += length_ups_status;
      uint32_t length_battery_status;
      arrToVar(length_battery_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_battery_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_battery_status-1]=0;
      this->battery_status = (char *)(inbuffer + offset-1);
      offset += length_battery_status;
      union {
        int16_t real;
        uint16_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_battery_current;
      u_battery_current.base = 0;
      u_battery_current.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_current.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_current = u_battery_current.real;
      offset += sizeof(this->battery_current);
      union {
        int16_t real;
        uint16_t base;
      } u_battery_capacity;
      u_battery_capacity.base = 0;
      u_battery_capacity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_capacity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_capacity = u_battery_capacity.real;
      offset += sizeof(this->battery_capacity);
      this->battery_warning =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_warning);
      this->battery_low =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_low);
      union {
        int16_t real;
        uint16_t base;
      } u_input_voltage;
      u_input_voltage.base = 0;
      u_input_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->input_voltage = u_input_voltage.real;
      offset += sizeof(this->input_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_input_current;
      u_input_current.base = 0;
      u_input_current.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_current.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->input_current = u_input_current.real;
      offset += sizeof(this->input_current);
      union {
        int16_t real;
        uint16_t base;
      } u_output_voltage;
      u_output_voltage.base = 0;
      u_output_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->output_voltage = u_output_voltage.real;
      offset += sizeof(this->output_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_output_current;
      u_output_current.base = 0;
      u_output_current.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_current.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->output_current = u_output_current.real;
      offset += sizeof(this->output_current);
      union {
        int16_t real;
        uint16_t base;
      } u_cell1_voltage;
      u_cell1_voltage.base = 0;
      u_cell1_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell1_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cell1_voltage = u_cell1_voltage.real;
      offset += sizeof(this->cell1_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell2_voltage;
      u_cell2_voltage.base = 0;
      u_cell2_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell2_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cell2_voltage = u_cell2_voltage.real;
      offset += sizeof(this->cell2_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell3_voltage;
      u_cell3_voltage.base = 0;
      u_cell3_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell3_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cell3_voltage = u_cell3_voltage.real;
      offset += sizeof(this->cell3_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell4_voltage;
      u_cell4_voltage.base = 0;
      u_cell4_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell4_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cell4_voltage = u_cell4_voltage.real;
      offset += sizeof(this->cell4_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell5_voltage;
      u_cell5_voltage.base = 0;
      u_cell5_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell5_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cell5_voltage = u_cell5_voltage.real;
      offset += sizeof(this->cell5_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_cell6_voltage;
      u_cell6_voltage.base = 0;
      u_cell6_voltage.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell6_voltage.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cell6_voltage = u_cell6_voltage.real;
      offset += sizeof(this->cell6_voltage);
     return offset;
    }

    const char * getType(){ return "vitulus_bringup/vitulus_ups"; };
    const char * getMD5(){ return "b62755b021b0543fc5e4775717df5161"; };

  };

}
#endif