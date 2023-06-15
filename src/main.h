#include <Arduino.h>
#include <SPI.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <PWM.h>  // !!!  require comment Timer2 safeinit in library ATimerDefs.cpp
#include <AutoPID.h>
#include <ros.h>
#include <vitulus_ups/vitulus_ups.h>
#include <vitulus_ups/power_status.h>
#include <vitulus_ups/power_values.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <EEPROM.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt64.h>



#define SIGNAL_LED    8  
#define POWER_SWITCH_SENSE    29  
#define OUT19V_SWITCH    5 
#define OUT19V_DC_DC_EN    6 
#define BATTERY_CHARGE_SWITCH    3 
#define BATTERY_DISCHARGE_SWITCH    4 
#define POWER_EN    31
#define BATT_OUTPUT_SWITCH   43
#define MOTORS_OUTPUT_SWITCH   42
#define FAN_PWM    9 
#define FAN2_PWM    10 
#define FAN_RPM    2 
#define FAN2_RPM    19 
#define ISET1_PWM    12 
#define ISET2_PWM    11
#define VIN_ADC    A0
#define BATTERY_VSENSE_ADC    A1
#define V19_VSENSE_ADC    A3 
#define ONE_WIRE_BUS_TEMP_SENSE_1 23
#define STAT_1 46
#define STAT_2 47
#define PG 44
#define TEMP_NTC A2         // Pin where the voltage divider is connected
#define TEMP_NTC_EXT A4         // Pin where the voltage divider is connected
#define TEMP_PWR 7        // 5V for the voltage divider

#define CHARGE_0A 0
#define CHARGE_1A 12
#define CHARGE_2A 25
#define CHARGE_3A 41
#define CHARGE_4A 52
#define CHARGE_5A 62
#define CHARGE_6A 72
#define CHARGE_FULL 100

#define PRECHARGE_TERM 50
#define PRECHARGE_TERM_MIN 30
#define PRECHARGE_TERM_MAX 100

#define PRECHARGE_MA_MIN 500    //[mA] min. value for precharge current
#define PRECHARGE_MA_MAX 1500   //[mA] max. value for precharge current
#define CHARGE_MA_MIN 0    //[mA] min. value for charge current
#define CHARGE_MA_MAX 6000   //[mA] max. value for charge current

// Charge current settings 
int charge_current_setpoint = CHARGE_1A;         // 0-100 
int charge_current_setpoint_running = CHARGE_1A;         // 0-100 
int charge_current_setpoint_standby = CHARGE_1A;         // 0-100 
int precharge_current_setpoint = PRECHARGE_TERM;         // 0-100 
int precharge_current_setpoint_running = PRECHARGE_TERM;         // 0-100 
int precharge_current_setpoint_standby = PRECHARGE_TERM;         // 0-100 
float frequency = 20000.0f; //frequency (in Hz) for charging PWM



#define BATT_EMPTY_VOLTAGE 18.5
#define BATT_FULL_VOLTAGE 24.0
#define BATT_LOW 20  // capacity when warn
#define BATT_EMPTY 5  // capacity when empty
#define BAT_SAMPLES 1200
#define ONLINE_MIN_VOLTAGE 25
int battery_capacity = -100;
unsigned long battery_capacity_samples = 0;
int battery_capacity_samples_count = 0;





// For NTC 
#define nominal_resistance 10000       //Nominal resistance at 25⁰C
#define nominal_temeprature 25   // temperature for nominal resistance (almost always 25⁰ C)
#define beta 3950  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000   //Value of  resistor used for the voltage divider
#define TEMP_SAMPLES 10  // Samples count for average value.
double tmp_in_buff = 0;
double tmp_ext_buff = 0;
int tmp_buff_count = 0;
double temp_in = 0;
double temp_ext = 0;
double temp_setpoint = 30.0;  // target temp for cooling 
double temp_ext_setpoint = 35.0;  // target temp for cooling 
#define TEMP_INT_PID_STEP 200  // PID update interval [ms]
#define TEMP_EXT_PID_STEP 200  // PID update interval [ms]


//////////////////// FAN
#define OUTPUT_MIN -255
#define OUTPUT_MAX  -9
#define OUTPUT_EXT_MIN  -255
#define OUTPUT_EXT_MAX  -9
#define KP 180
#define KI 1.6
#define KD 30
int fanSpeed = 255;
int fan_rpm = 0;
int fan_rpm_count = 0;
double outputVal;
double outputExtTempVal;
int fan2Speed = 255;
int fan2_rpm = 0;
int fan2_rpm_count = 0;


float   charge_current, 
        discharge_current, 
        output19v_current, 
        out19_v, 
        bat_v, 
        input_v,
        input_i, 
        charge_i, 
        discharge_i,
        out19_i,
        ds_temp,
        pg_buffer = 0;

 bool   stat_1,
        stat_2,
        pg,
        pwr_switch = 1;

int pg_counter = 0;


int last_input_status = 0; // 0=OFFLINE, 1=ONLINE, 2=FAIL
int input_status = 0; // 0=OFFLINE, 1=ONLINE, 2=FAIL
#define OFFLINE 0
#define ONLINE  1 
#define FAIL 2

int last_charger_status = 0; // 0=DISCHARGING, 1=CHARGED, 2=CHARGING
int charger_status = 0; // 0=DISCHARGING, 1=CHARGED, 2=CHARGING
#define DISCHARGING 0
#define CHARGED  1
#define CHARGING 2



unsigned long   lastmillis_1Hz, 
                lastmillis_20Hz, 
                lastmillis_3s, 
                lastmillis_2Hz,
                lastmillis_200ms, 
                lastmillis_led, 
                lastmillis_pwr_button;
#define LED_CHARGING_ON        3000
#define LED_CHARGING_OFF       30
#define LED_CHARGED_ON         3000
#define LED_CHARGED_OFF        0
#define LED_DISCHARGING_ON     1000
#define LED_DISCHARGING_OFF    100
unsigned long  lastmillis_led_interval = LED_CHARGING_ON;
bool signal_led = 0;
bool led_state = 0;



int last_button_status;


int power_module_status = 0; // 0=STARTUP,  1=RUNNING, 2=STANDBY
#define STARTUP    0
#define RUNNING    1
#define STANDBY    2


bool output19_switch = 0;  // 0=off , 1=on
bool dc19_enable = 1;  // 1=off , 0=on
bool battery_discharge_switch = 1;  // 0=off , 1=on
bool battery_charge_switch = 0;  // 0=off , 1=on
bool power_enable_switch = 1;  // 0=off , 1=on
bool battery_output_switch = 0;  // 0=off , 1=on
bool motors_output_switch = 0;  // 0=off , 1=on

int charge_state = 0;  // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
bool standby_timeout_running = false;
unsigned long   lastmillis_timeout;

////////////////////// TURN OFF AFTER TIME IN ms WHEN NOT OPERATE AND NOT SLEEP
unsigned long standby_timeout_discharging = 3600000; // 3600000ms = 1h

////////////////////// MAKE SLEEP AND WAKE THE ROBOT AFTER TIME IN ms OR WHILE CHARGING
unsigned long  sleep_wait_before_standby_timer; // Wait before standby. Let the robot shutdown by it self  120000ms = 2m
unsigned long  sleep_wait_before_standby_millis = 120000; // Wait before standby. Let the robot shutdown by it self  120000ms = 2m

bool sleep_status = false;  // Control sleep for the time interval.
unsigned long  sleep_time_timer; // 3600000ms = 1h
unsigned long  sleep_time_millis = 240000; // How long to sleep 3600000ms = 1h

bool sleep_charged_status = false;  // Control sleep while charging
unsigned long  sleep_charged_timer; // 3600000ms = 1h
unsigned long  sleep_wait_charged_offset_millis = 120000; // Wait after charged before startup  120000ms = 2m



//////////////////// EEPROM  addr and init check
int8_t check_eeprom = 0;
int check_eeprom_addr = 0; 
int charge_current_setpoint_addr = sizeof(int8_t);
int precharge_current_setpoint_addr =  charge_current_setpoint_addr + sizeof(int);
int temp_setpoint_addr =  precharge_current_setpoint_addr + sizeof(int);
int standby_timeout_discharging_addr =  temp_setpoint_addr + sizeof(double);
int sleep_time_millis_addr =  standby_timeout_discharging_addr + sizeof(unsigned long);
int sleep_wait_before_standby_millis_addr =  sleep_time_millis_addr + sizeof(unsigned long);
int sleep_wait_charged_offset_millis_addr =  sleep_wait_before_standby_millis_addr + sizeof(unsigned long);
int charge_current_setpoint_running_addr =  sleep_wait_charged_offset_millis_addr + sizeof(unsigned long);
int charge_current_setpoint_standby_addr =  charge_current_setpoint_running_addr + sizeof(int);
int precharge_current_setpoint_running_addr =  charge_current_setpoint_standby_addr + sizeof(int);
int precharge_current_setpoint_standby_addr =  precharge_current_setpoint_running_addr + sizeof(int);
int temp_ext_setpoint_addr =  precharge_current_setpoint_standby_addr + sizeof(int);



float get_amps(float, float);
void get_ADS();
void get_bq24610();
void get_voltage();
void get_ntc();
void calculate_ntc();
void get_ds18b20();
void get_pwrswitch();
void fanRpmInt();
void fan2RpmInt();
void get_input_status();
void get_charger_status();
void set_led();
void batteryCapacity();
void setChargingSwitch();
void overDischargeOff();
void standbyTimeout();
void set_robot_poweron();
void set_standby();
void set_shutdown();
void set_charge_current();
void check_sleep();
void printData();
void batteryCapacity();


void rosMsgUps();
// void setChargeCurrent(const std_msgs::Int8& charge_current_msg);
void rosMsgPowerStatus();
void rosMsgPowerValues();
void set_precharge_current_running(const std_msgs::Int16& set_precharge_current_running_msg);
void set_precharge_current_standby(const std_msgs::Int16& set_precharge_current_standby_msg);
void set_charge_current_running(const std_msgs::Int16& set_charge_current_running_msg);
void set_charge_current_standby(const std_msgs::Int16& set_charge_current_standby_msg);
void set_charge_switch(const std_msgs::Bool& set_charge_switch_msg);
void set_discharge_switch(const std_msgs::Bool& set_discharge_switch_msg);
void set_motor_switch(const std_msgs::Bool& set_motor_switch_msg);
void set_bat_out_switch(const std_msgs::Bool& set_bat_out_switch_msg);
void set_19v_out_switch(const std_msgs::Bool& set_19v_out_switch_msg);
void set_temp_setpoint(const std_msgs::Float64& set_temp_setpoint_msg);
void set_robot_sleep(const std_msgs::Bool& set_robot_sleep_msg);
void set_sleep_until_charged(const std_msgs::Bool& set_sleep_until_charged_msg);
void set_standby_timeout_discharging(const std_msgs::UInt64& set_standby_timeout_discharging_msg);
void set_sleep_time(const std_msgs::UInt64& set_sleep_time_msg);
void set_sleep_wait_before_standby(const std_msgs::UInt64& set_sleep_wait_before_standby_msg);
void set_sleep_wait_charged_offset(const std_msgs::UInt64& set_sleep_wait_charged_offset_msg);
void set_temp2_setpoint(const std_msgs::Float64& set_temp2_setpoint_msg);
void play_melody(const std_msgs::Int16& play_melody_msg);

ros::NodeHandle nh;

vitulus_ups::vitulus_ups vitulus_ups_msg;
ros::Publisher pub_ups("ups", &vitulus_ups_msg);
vitulus_ups::power_status power_status_msg;
ros::Publisher pub_power_status("power_status", &power_status_msg);
vitulus_ups::power_values power_values_msg;
ros::Publisher pub_power_values("power_values", &power_values_msg);

// ros::Subscriber<std_msgs::Int8> sub_set_charge_current("set_charge_current", setChargeCurrent );
ros::Subscriber<std_msgs::Int16> sub_set_precharge_current_running("set_precharge_current_running", set_precharge_current_running );
ros::Subscriber<std_msgs::Int16> sub_set_precharge_current_standby("set_precharge_current_standby", set_precharge_current_standby );
ros::Subscriber<std_msgs::Int16> sub_set_charge_current_running("set_charge_current_running", set_charge_current_running );
ros::Subscriber<std_msgs::Int16> sub_set_charge_current_standby("set_charge_current_standby", set_charge_current_standby );
ros::Subscriber<std_msgs::Bool> sub_set_charge_switch("set_charge_switch", set_charge_switch );
ros::Subscriber<std_msgs::Bool> sub_set_discharge_switch("set_discharge_switch", set_discharge_switch );
ros::Subscriber<std_msgs::Bool> sub_set_motor_switch("set_motor_switch", set_motor_switch );
ros::Subscriber<std_msgs::Bool> sub_set_bat_out_switch("set_bat_out_switch", set_bat_out_switch );
ros::Subscriber<std_msgs::Bool> sub_set_19v_out_switch("set_19v_out_switch", set_19v_out_switch );
ros::Subscriber<std_msgs::Float64> sub_set_temp_setpoint("set_temp_setpoint", set_temp_setpoint );
ros::Subscriber<std_msgs::Bool> sub_set_robot_sleep("set_robot_sleep", set_robot_sleep );
ros::Subscriber<std_msgs::Bool> sub_set_sleep_until_charged("set_sleep_until_charged", set_sleep_until_charged );
ros::Subscriber<std_msgs::UInt64> sub_set_standby_timeout_discharg("set_standby_timeout_discharging", set_standby_timeout_discharging );
ros::Subscriber<std_msgs::UInt64> sub_set_sleep_time("set_sleep_time", set_sleep_time );
ros::Subscriber<std_msgs::UInt64> sub_set_sleep_wait_before_standby("set_sleep_wait_before_standby", set_sleep_wait_before_standby );
ros::Subscriber<std_msgs::UInt64> sub_set_sleep_wait_charged_offset("set_sleep_wait_charged_offset", set_sleep_wait_charged_offset );
ros::Subscriber<std_msgs::Float64> sub_set_temp2_setpoint("set_temp2_setpoint", set_temp2_setpoint );
ros::Subscriber<std_msgs::Int16> sub_play_melody("play_melody", play_melody );

