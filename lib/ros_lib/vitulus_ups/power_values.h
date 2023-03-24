#ifndef _ROS_vitulus_ups_power_values_h
#define _ROS_vitulus_ups_power_values_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace vitulus_ups
{

  class power_values : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _input_voltage_type;
      _input_voltage_type input_voltage;
      typedef float _input_current_type;
      _input_current_type input_current;
      typedef float _input_power_type;
      _input_power_type input_power;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _battery_current_type;
      _battery_current_type battery_current;
      typedef float _battery_power_type;
      _battery_power_type battery_power;
      typedef float _battery_charge_current_type;
      _battery_charge_current_type battery_charge_current;
      typedef float _battery_charge_power_type;
      _battery_charge_power_type battery_charge_power;
      typedef float _battery_discharge_current_type;
      _battery_discharge_current_type battery_discharge_current;
      typedef float _battery_discharge_power_type;
      _battery_discharge_power_type battery_discharge_power;
      typedef float _out19_voltage_type;
      _out19_voltage_type out19_voltage;
      typedef float _out19_current_type;
      _out19_current_type out19_current;
      typedef float _out19_power_type;
      _out19_power_type out19_power;

    power_values():
      header(),
      input_voltage(0),
      input_current(0),
      input_power(0),
      battery_voltage(0),
      battery_current(0),
      battery_power(0),
      battery_charge_current(0),
      battery_charge_power(0),
      battery_discharge_current(0),
      battery_discharge_power(0),
      out19_voltage(0),
      out19_current(0),
      out19_power(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_input_voltage;
      u_input_voltage.real = this->input_voltage;
      *(outbuffer + offset + 0) = (u_input_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_voltage);
      union {
        float real;
        uint32_t base;
      } u_input_current;
      u_input_current.real = this->input_current;
      *(outbuffer + offset + 0) = (u_input_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_current);
      union {
        float real;
        uint32_t base;
      } u_input_power;
      u_input_power.real = this->input_power;
      *(outbuffer + offset + 0) = (u_input_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_power);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_current;
      u_battery_current.real = this->battery_current;
      *(outbuffer + offset + 0) = (u_battery_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_current);
      union {
        float real;
        uint32_t base;
      } u_battery_power;
      u_battery_power.real = this->battery_power;
      *(outbuffer + offset + 0) = (u_battery_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_power);
      union {
        float real;
        uint32_t base;
      } u_battery_charge_current;
      u_battery_charge_current.real = this->battery_charge_current;
      *(outbuffer + offset + 0) = (u_battery_charge_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_charge_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_charge_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_charge_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_charge_current);
      union {
        float real;
        uint32_t base;
      } u_battery_charge_power;
      u_battery_charge_power.real = this->battery_charge_power;
      *(outbuffer + offset + 0) = (u_battery_charge_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_charge_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_charge_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_charge_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_charge_power);
      union {
        float real;
        uint32_t base;
      } u_battery_discharge_current;
      u_battery_discharge_current.real = this->battery_discharge_current;
      *(outbuffer + offset + 0) = (u_battery_discharge_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_discharge_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_discharge_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_discharge_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_discharge_current);
      union {
        float real;
        uint32_t base;
      } u_battery_discharge_power;
      u_battery_discharge_power.real = this->battery_discharge_power;
      *(outbuffer + offset + 0) = (u_battery_discharge_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_discharge_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_discharge_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_discharge_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_discharge_power);
      union {
        float real;
        uint32_t base;
      } u_out19_voltage;
      u_out19_voltage.real = this->out19_voltage;
      *(outbuffer + offset + 0) = (u_out19_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_out19_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_out19_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_out19_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->out19_voltage);
      union {
        float real;
        uint32_t base;
      } u_out19_current;
      u_out19_current.real = this->out19_current;
      *(outbuffer + offset + 0) = (u_out19_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_out19_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_out19_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_out19_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->out19_current);
      union {
        float real;
        uint32_t base;
      } u_out19_power;
      u_out19_power.real = this->out19_power;
      *(outbuffer + offset + 0) = (u_out19_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_out19_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_out19_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_out19_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->out19_power);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_input_voltage;
      u_input_voltage.base = 0;
      u_input_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_voltage = u_input_voltage.real;
      offset += sizeof(this->input_voltage);
      union {
        float real;
        uint32_t base;
      } u_input_current;
      u_input_current.base = 0;
      u_input_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_current = u_input_current.real;
      offset += sizeof(this->input_current);
      union {
        float real;
        uint32_t base;
      } u_input_power;
      u_input_power.base = 0;
      u_input_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_power = u_input_power.real;
      offset += sizeof(this->input_power);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_current;
      u_battery_current.base = 0;
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_current = u_battery_current.real;
      offset += sizeof(this->battery_current);
      union {
        float real;
        uint32_t base;
      } u_battery_power;
      u_battery_power.base = 0;
      u_battery_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_power = u_battery_power.real;
      offset += sizeof(this->battery_power);
      union {
        float real;
        uint32_t base;
      } u_battery_charge_current;
      u_battery_charge_current.base = 0;
      u_battery_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_charge_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_charge_current = u_battery_charge_current.real;
      offset += sizeof(this->battery_charge_current);
      union {
        float real;
        uint32_t base;
      } u_battery_charge_power;
      u_battery_charge_power.base = 0;
      u_battery_charge_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_charge_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_charge_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_charge_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_charge_power = u_battery_charge_power.real;
      offset += sizeof(this->battery_charge_power);
      union {
        float real;
        uint32_t base;
      } u_battery_discharge_current;
      u_battery_discharge_current.base = 0;
      u_battery_discharge_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_discharge_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_discharge_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_discharge_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_discharge_current = u_battery_discharge_current.real;
      offset += sizeof(this->battery_discharge_current);
      union {
        float real;
        uint32_t base;
      } u_battery_discharge_power;
      u_battery_discharge_power.base = 0;
      u_battery_discharge_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_discharge_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_discharge_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_discharge_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_discharge_power = u_battery_discharge_power.real;
      offset += sizeof(this->battery_discharge_power);
      union {
        float real;
        uint32_t base;
      } u_out19_voltage;
      u_out19_voltage.base = 0;
      u_out19_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_out19_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_out19_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_out19_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->out19_voltage = u_out19_voltage.real;
      offset += sizeof(this->out19_voltage);
      union {
        float real;
        uint32_t base;
      } u_out19_current;
      u_out19_current.base = 0;
      u_out19_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_out19_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_out19_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_out19_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->out19_current = u_out19_current.real;
      offset += sizeof(this->out19_current);
      union {
        float real;
        uint32_t base;
      } u_out19_power;
      u_out19_power.base = 0;
      u_out19_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_out19_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_out19_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_out19_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->out19_power = u_out19_power.real;
      offset += sizeof(this->out19_power);
     return offset;
    }

    virtual const char * getType() override { return "vitulus_ups/power_values"; };
    virtual const char * getMD5() override { return "0b62cd7dd4b8bc579c5cd2c9d9cb9a0d"; };

  };

}
#endif
