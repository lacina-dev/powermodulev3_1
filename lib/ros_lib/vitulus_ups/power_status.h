#ifndef _ROS_vitulus_ups_power_status_h
#define _ROS_vitulus_ups_power_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace vitulus_ups
{

  class power_status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _version_type;
      _version_type version;
      typedef const char* _charger_status_type;
      _charger_status_type charger_status;
      typedef const char* _supply_status_type;
      _supply_status_type supply_status;
      typedef const char* _module_status_type;
      _module_status_type module_status;
      typedef int16_t _battery_capacity_type;
      _battery_capacity_type battery_capacity;
      typedef int16_t _charge_current_setpoint_run_type;
      _charge_current_setpoint_run_type charge_current_setpoint_run;
      typedef int16_t _charge_current_setpoint_standby_type;
      _charge_current_setpoint_standby_type charge_current_setpoint_standby;
      typedef int16_t _precharge_current_setpoint_run_type;
      _precharge_current_setpoint_run_type precharge_current_setpoint_run;
      typedef int16_t _precharge_current_setpoint_standby_type;
      _precharge_current_setpoint_standby_type precharge_current_setpoint_standby;
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
      typedef float _temp_setpoint_type;
      _temp_setpoint_type temp_setpoint;
      typedef float _temp2_setpoint_type;
      _temp2_setpoint_type temp2_setpoint;
      typedef float _temp_type;
      _temp_type temp;
      typedef float _temp2_type;
      _temp2_type temp2;
      typedef int16_t _fan_rpm_type;
      _fan_rpm_type fan_rpm;
      typedef int16_t _fan2_rpm_type;
      _fan2_rpm_type fan2_rpm;
      typedef bool _charge_switch_type;
      _charge_switch_type charge_switch;
      typedef bool _discharge_switch_type;
      _discharge_switch_type discharge_switch;
      typedef bool _bat_out_switch_type;
      _bat_out_switch_type bat_out_switch;
      typedef bool _motor_out_switch_type;
      _motor_out_switch_type motor_out_switch;
      typedef bool _out19v_switch_type;
      _out19v_switch_type out19v_switch;
      typedef bool _enable19v_switch_type;
      _enable19v_switch_type enable19v_switch;
      typedef uint64_t _standby_timeout_type;
      _standby_timeout_type standby_timeout;
      typedef bool _sleep_status_type;
      _sleep_status_type sleep_status;
      typedef bool _sleep_charged_status_type;
      _sleep_charged_status_type sleep_charged_status;
      typedef uint64_t _sleep_wait_standby_type;
      _sleep_wait_standby_type sleep_wait_standby;
      typedef uint64_t _sleep_time_interval_type;
      _sleep_time_interval_type sleep_time_interval;
      typedef uint64_t _sleep_wait_charged_type;
      _sleep_wait_charged_type sleep_wait_charged;
      typedef const char* _debug_info_type;
      _debug_info_type debug_info;

    power_status():
      header(),
      version(""),
      charger_status(""),
      supply_status(""),
      module_status(""),
      battery_capacity(0),
      charge_current_setpoint_run(0),
      charge_current_setpoint_standby(0),
      precharge_current_setpoint_run(0),
      precharge_current_setpoint_standby(0),
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
      out19_power(0),
      temp_setpoint(0),
      temp2_setpoint(0),
      temp(0),
      temp2(0),
      fan_rpm(0),
      fan2_rpm(0),
      charge_switch(0),
      discharge_switch(0),
      bat_out_switch(0),
      motor_out_switch(0),
      out19v_switch(0),
      enable19v_switch(0),
      standby_timeout(0),
      sleep_status(0),
      sleep_charged_status(0),
      sleep_wait_standby(0),
      sleep_time_interval(0),
      sleep_wait_charged(0),
      debug_info("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_version = strlen(this->version);
      varToArr(outbuffer + offset, length_version);
      offset += 4;
      memcpy(outbuffer + offset, this->version, length_version);
      offset += length_version;
      uint32_t length_charger_status = strlen(this->charger_status);
      varToArr(outbuffer + offset, length_charger_status);
      offset += 4;
      memcpy(outbuffer + offset, this->charger_status, length_charger_status);
      offset += length_charger_status;
      uint32_t length_supply_status = strlen(this->supply_status);
      varToArr(outbuffer + offset, length_supply_status);
      offset += 4;
      memcpy(outbuffer + offset, this->supply_status, length_supply_status);
      offset += length_supply_status;
      uint32_t length_module_status = strlen(this->module_status);
      varToArr(outbuffer + offset, length_module_status);
      offset += 4;
      memcpy(outbuffer + offset, this->module_status, length_module_status);
      offset += length_module_status;
      union {
        int16_t real;
        uint16_t base;
      } u_battery_capacity;
      u_battery_capacity.real = this->battery_capacity;
      *(outbuffer + offset + 0) = (u_battery_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_capacity.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->battery_capacity);
      union {
        int16_t real;
        uint16_t base;
      } u_charge_current_setpoint_run;
      u_charge_current_setpoint_run.real = this->charge_current_setpoint_run;
      *(outbuffer + offset + 0) = (u_charge_current_setpoint_run.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge_current_setpoint_run.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->charge_current_setpoint_run);
      union {
        int16_t real;
        uint16_t base;
      } u_charge_current_setpoint_standby;
      u_charge_current_setpoint_standby.real = this->charge_current_setpoint_standby;
      *(outbuffer + offset + 0) = (u_charge_current_setpoint_standby.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge_current_setpoint_standby.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->charge_current_setpoint_standby);
      union {
        int16_t real;
        uint16_t base;
      } u_precharge_current_setpoint_run;
      u_precharge_current_setpoint_run.real = this->precharge_current_setpoint_run;
      *(outbuffer + offset + 0) = (u_precharge_current_setpoint_run.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_precharge_current_setpoint_run.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->precharge_current_setpoint_run);
      union {
        int16_t real;
        uint16_t base;
      } u_precharge_current_setpoint_standby;
      u_precharge_current_setpoint_standby.real = this->precharge_current_setpoint_standby;
      *(outbuffer + offset + 0) = (u_precharge_current_setpoint_standby.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_precharge_current_setpoint_standby.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->precharge_current_setpoint_standby);
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
      union {
        float real;
        uint32_t base;
      } u_temp_setpoint;
      u_temp_setpoint.real = this->temp_setpoint;
      *(outbuffer + offset + 0) = (u_temp_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_setpoint);
      union {
        float real;
        uint32_t base;
      } u_temp2_setpoint;
      u_temp2_setpoint.real = this->temp2_setpoint;
      *(outbuffer + offset + 0) = (u_temp2_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp2_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp2_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp2_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp2_setpoint);
      union {
        float real;
        uint32_t base;
      } u_temp;
      u_temp.real = this->temp;
      *(outbuffer + offset + 0) = (u_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp);
      union {
        float real;
        uint32_t base;
      } u_temp2;
      u_temp2.real = this->temp2;
      *(outbuffer + offset + 0) = (u_temp2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp2);
      union {
        int16_t real;
        uint16_t base;
      } u_fan_rpm;
      u_fan_rpm.real = this->fan_rpm;
      *(outbuffer + offset + 0) = (u_fan_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fan_rpm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fan_rpm);
      union {
        int16_t real;
        uint16_t base;
      } u_fan2_rpm;
      u_fan2_rpm.real = this->fan2_rpm;
      *(outbuffer + offset + 0) = (u_fan2_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fan2_rpm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fan2_rpm);
      union {
        bool real;
        uint8_t base;
      } u_charge_switch;
      u_charge_switch.real = this->charge_switch;
      *(outbuffer + offset + 0) = (u_charge_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->charge_switch);
      union {
        bool real;
        uint8_t base;
      } u_discharge_switch;
      u_discharge_switch.real = this->discharge_switch;
      *(outbuffer + offset + 0) = (u_discharge_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->discharge_switch);
      union {
        bool real;
        uint8_t base;
      } u_bat_out_switch;
      u_bat_out_switch.real = this->bat_out_switch;
      *(outbuffer + offset + 0) = (u_bat_out_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bat_out_switch);
      union {
        bool real;
        uint8_t base;
      } u_motor_out_switch;
      u_motor_out_switch.real = this->motor_out_switch;
      *(outbuffer + offset + 0) = (u_motor_out_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_out_switch);
      union {
        bool real;
        uint8_t base;
      } u_out19v_switch;
      u_out19v_switch.real = this->out19v_switch;
      *(outbuffer + offset + 0) = (u_out19v_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->out19v_switch);
      union {
        bool real;
        uint8_t base;
      } u_enable19v_switch;
      u_enable19v_switch.real = this->enable19v_switch;
      *(outbuffer + offset + 0) = (u_enable19v_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable19v_switch);
      *(outbuffer + offset + 0) = (this->standby_timeout >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->standby_timeout >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->standby_timeout >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->standby_timeout >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->standby_timeout >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->standby_timeout >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->standby_timeout >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->standby_timeout >> (8 * 7)) & 0xFF;
      offset += sizeof(this->standby_timeout);
      union {
        bool real;
        uint8_t base;
      } u_sleep_status;
      u_sleep_status.real = this->sleep_status;
      *(outbuffer + offset + 0) = (u_sleep_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sleep_status);
      union {
        bool real;
        uint8_t base;
      } u_sleep_charged_status;
      u_sleep_charged_status.real = this->sleep_charged_status;
      *(outbuffer + offset + 0) = (u_sleep_charged_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sleep_charged_status);
      *(outbuffer + offset + 0) = (this->sleep_wait_standby >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sleep_wait_standby >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sleep_wait_standby >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sleep_wait_standby >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->sleep_wait_standby >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->sleep_wait_standby >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->sleep_wait_standby >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->sleep_wait_standby >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sleep_wait_standby);
      *(outbuffer + offset + 0) = (this->sleep_time_interval >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sleep_time_interval >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sleep_time_interval >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sleep_time_interval >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->sleep_time_interval >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->sleep_time_interval >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->sleep_time_interval >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->sleep_time_interval >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sleep_time_interval);
      *(outbuffer + offset + 0) = (this->sleep_wait_charged >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sleep_wait_charged >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sleep_wait_charged >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sleep_wait_charged >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->sleep_wait_charged >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->sleep_wait_charged >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->sleep_wait_charged >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->sleep_wait_charged >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sleep_wait_charged);
      uint32_t length_debug_info = strlen(this->debug_info);
      varToArr(outbuffer + offset, length_debug_info);
      offset += 4;
      memcpy(outbuffer + offset, this->debug_info, length_debug_info);
      offset += length_debug_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_version;
      arrToVar(length_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_version-1]=0;
      this->version = (char *)(inbuffer + offset-1);
      offset += length_version;
      uint32_t length_charger_status;
      arrToVar(length_charger_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_charger_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_charger_status-1]=0;
      this->charger_status = (char *)(inbuffer + offset-1);
      offset += length_charger_status;
      uint32_t length_supply_status;
      arrToVar(length_supply_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_supply_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_supply_status-1]=0;
      this->supply_status = (char *)(inbuffer + offset-1);
      offset += length_supply_status;
      uint32_t length_module_status;
      arrToVar(length_module_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module_status-1]=0;
      this->module_status = (char *)(inbuffer + offset-1);
      offset += length_module_status;
      union {
        int16_t real;
        uint16_t base;
      } u_battery_capacity;
      u_battery_capacity.base = 0;
      u_battery_capacity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_capacity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_capacity = u_battery_capacity.real;
      offset += sizeof(this->battery_capacity);
      union {
        int16_t real;
        uint16_t base;
      } u_charge_current_setpoint_run;
      u_charge_current_setpoint_run.base = 0;
      u_charge_current_setpoint_run.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge_current_setpoint_run.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->charge_current_setpoint_run = u_charge_current_setpoint_run.real;
      offset += sizeof(this->charge_current_setpoint_run);
      union {
        int16_t real;
        uint16_t base;
      } u_charge_current_setpoint_standby;
      u_charge_current_setpoint_standby.base = 0;
      u_charge_current_setpoint_standby.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge_current_setpoint_standby.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->charge_current_setpoint_standby = u_charge_current_setpoint_standby.real;
      offset += sizeof(this->charge_current_setpoint_standby);
      union {
        int16_t real;
        uint16_t base;
      } u_precharge_current_setpoint_run;
      u_precharge_current_setpoint_run.base = 0;
      u_precharge_current_setpoint_run.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_precharge_current_setpoint_run.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->precharge_current_setpoint_run = u_precharge_current_setpoint_run.real;
      offset += sizeof(this->precharge_current_setpoint_run);
      union {
        int16_t real;
        uint16_t base;
      } u_precharge_current_setpoint_standby;
      u_precharge_current_setpoint_standby.base = 0;
      u_precharge_current_setpoint_standby.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_precharge_current_setpoint_standby.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->precharge_current_setpoint_standby = u_precharge_current_setpoint_standby.real;
      offset += sizeof(this->precharge_current_setpoint_standby);
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
      union {
        float real;
        uint32_t base;
      } u_temp_setpoint;
      u_temp_setpoint.base = 0;
      u_temp_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_setpoint = u_temp_setpoint.real;
      offset += sizeof(this->temp_setpoint);
      union {
        float real;
        uint32_t base;
      } u_temp2_setpoint;
      u_temp2_setpoint.base = 0;
      u_temp2_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp2_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp2_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp2_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp2_setpoint = u_temp2_setpoint.real;
      offset += sizeof(this->temp2_setpoint);
      union {
        float real;
        uint32_t base;
      } u_temp;
      u_temp.base = 0;
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp = u_temp.real;
      offset += sizeof(this->temp);
      union {
        float real;
        uint32_t base;
      } u_temp2;
      u_temp2.base = 0;
      u_temp2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp2 = u_temp2.real;
      offset += sizeof(this->temp2);
      union {
        int16_t real;
        uint16_t base;
      } u_fan_rpm;
      u_fan_rpm.base = 0;
      u_fan_rpm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fan_rpm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fan_rpm = u_fan_rpm.real;
      offset += sizeof(this->fan_rpm);
      union {
        int16_t real;
        uint16_t base;
      } u_fan2_rpm;
      u_fan2_rpm.base = 0;
      u_fan2_rpm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fan2_rpm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fan2_rpm = u_fan2_rpm.real;
      offset += sizeof(this->fan2_rpm);
      union {
        bool real;
        uint8_t base;
      } u_charge_switch;
      u_charge_switch.base = 0;
      u_charge_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->charge_switch = u_charge_switch.real;
      offset += sizeof(this->charge_switch);
      union {
        bool real;
        uint8_t base;
      } u_discharge_switch;
      u_discharge_switch.base = 0;
      u_discharge_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->discharge_switch = u_discharge_switch.real;
      offset += sizeof(this->discharge_switch);
      union {
        bool real;
        uint8_t base;
      } u_bat_out_switch;
      u_bat_out_switch.base = 0;
      u_bat_out_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bat_out_switch = u_bat_out_switch.real;
      offset += sizeof(this->bat_out_switch);
      union {
        bool real;
        uint8_t base;
      } u_motor_out_switch;
      u_motor_out_switch.base = 0;
      u_motor_out_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motor_out_switch = u_motor_out_switch.real;
      offset += sizeof(this->motor_out_switch);
      union {
        bool real;
        uint8_t base;
      } u_out19v_switch;
      u_out19v_switch.base = 0;
      u_out19v_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->out19v_switch = u_out19v_switch.real;
      offset += sizeof(this->out19v_switch);
      union {
        bool real;
        uint8_t base;
      } u_enable19v_switch;
      u_enable19v_switch.base = 0;
      u_enable19v_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable19v_switch = u_enable19v_switch.real;
      offset += sizeof(this->enable19v_switch);
      this->standby_timeout =  ((uint64_t) (*(inbuffer + offset)));
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->standby_timeout |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->standby_timeout);
      union {
        bool real;
        uint8_t base;
      } u_sleep_status;
      u_sleep_status.base = 0;
      u_sleep_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sleep_status = u_sleep_status.real;
      offset += sizeof(this->sleep_status);
      union {
        bool real;
        uint8_t base;
      } u_sleep_charged_status;
      u_sleep_charged_status.base = 0;
      u_sleep_charged_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sleep_charged_status = u_sleep_charged_status.real;
      offset += sizeof(this->sleep_charged_status);
      this->sleep_wait_standby =  ((uint64_t) (*(inbuffer + offset)));
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->sleep_wait_standby |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->sleep_wait_standby);
      this->sleep_time_interval =  ((uint64_t) (*(inbuffer + offset)));
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->sleep_time_interval |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->sleep_time_interval);
      this->sleep_wait_charged =  ((uint64_t) (*(inbuffer + offset)));
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->sleep_wait_charged |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->sleep_wait_charged);
      uint32_t length_debug_info;
      arrToVar(length_debug_info, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_debug_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_debug_info-1]=0;
      this->debug_info = (char *)(inbuffer + offset-1);
      offset += length_debug_info;
     return offset;
    }

    virtual const char * getType() override { return "vitulus_ups/power_status"; };
    virtual const char * getMD5() override { return "725d0857e003e432590ab617cdc61011"; };

  };

}
#endif
