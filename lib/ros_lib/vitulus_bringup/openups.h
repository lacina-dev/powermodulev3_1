#ifndef _ROS_vitulus_bringup_openups_h
#define _ROS_vitulus_bringup_openups_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace vitulus_bringup
{

  class openups : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _driver_type;
      _driver_type driver;
      typedef const char* _ups_status_type;
      _ups_status_type ups_status;
      typedef const char* _ups_model_type;
      _ups_model_type ups_model;
      typedef uint8_t _battery_charge_type;
      _battery_charge_type battery_charge;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _battery_current_type;
      _battery_current_type battery_current;
      typedef uint8_t _battery_capacity_type;
      _battery_capacity_type battery_capacity;
      typedef uint8_t _battery_warning_type;
      _battery_warning_type battery_warning;
      typedef uint8_t _battery_low_type;
      _battery_low_type battery_low;
      typedef float _battery_temperature_type;
      _battery_temperature_type battery_temperature;
      typedef float _input_voltage_type;
      _input_voltage_type input_voltage;
      typedef float _input_current_type;
      _input_current_type input_current;
      typedef float _output_voltage_type;
      _output_voltage_type output_voltage;
      typedef float _output_current_type;
      _output_current_type output_current;

    openups():
      header(),
      driver(""),
      ups_status(""),
      ups_model(""),
      battery_charge(0),
      battery_voltage(0),
      battery_current(0),
      battery_capacity(0),
      battery_warning(0),
      battery_low(0),
      battery_temperature(0),
      input_voltage(0),
      input_current(0),
      output_voltage(0),
      output_current(0)
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
      uint32_t length_ups_model = strlen(this->ups_model);
      varToArr(outbuffer + offset, length_ups_model);
      offset += 4;
      memcpy(outbuffer + offset, this->ups_model, length_ups_model);
      offset += length_ups_model;
      *(outbuffer + offset + 0) = (this->battery_charge >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_charge);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery_current);
      *(outbuffer + offset + 0) = (this->battery_capacity >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_capacity);
      *(outbuffer + offset + 0) = (this->battery_warning >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_warning);
      *(outbuffer + offset + 0) = (this->battery_low >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_low);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery_temperature);
      offset += serializeAvrFloat64(outbuffer + offset, this->input_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->input_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->output_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->output_current);
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
      uint32_t length_ups_model;
      arrToVar(length_ups_model, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ups_model; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ups_model-1]=0;
      this->ups_model = (char *)(inbuffer + offset-1);
      offset += length_ups_model;
      this->battery_charge =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_charge);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery_current));
      this->battery_capacity =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_capacity);
      this->battery_warning =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_warning);
      this->battery_low =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_low);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery_temperature));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->input_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->input_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->output_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->output_current));
     return offset;
    }

    const char * getType(){ return "vitulus_bringup/openups"; };
    const char * getMD5(){ return "1e6e4e077413661eba808ba4a07585d7"; };

  };

}
#endif