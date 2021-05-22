#include <Arduino.h>
#include <SPI.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <PWM.h>
#include <AutoPID.h>
#include <ros.h>
#include <vitulus_ups/vitulus_ups.h>
#include <std_msgs/Int8.h>


#define SIGNAL_LED    8  
#define POWER_SWITCH_SENSE    29  
#define OUT19V_SWITCH    5 
#define OUT19V_DC_DC_EN    6 
#define BATTERY_CHARGE_SWITCH    3 
#define BATTERY_DISCHARGE_SWITCH    4 
#define FAN_PWM    9 
#define FAN_RPM    2 
#define ISET1_PWM    12 
#define ISET2_PWM    11
#define VIN_ADC    A0
#define BATTERY_VSENSE_ADC    A1 
#define ONE_WIRE_BUS_TEMP_SENSE_1 23
#define ONE_WIRE_BUS_TEMP_SENSE_2 24

#define CHARGE_0A 0
#define CHARGE_1A 15
#define CHARGE_2A 29
#define CHARGE_4A 61
#define CHARGE_6A 84
#define CHARGE_FULL 100

#define PRECHARGE_TERM 25


#define BATT_EMPTY_VOLTAGE 19.5
#define BATT_FULL_VOLTAGE 24.0
#define BATT_LOW 30  // capacity when warn
#define BATT_EMPTY 20  // capacity when empty


#define SAMPLES 20.0

#define ONLINE_MIN_VOLTAGE 25

#define OUTPUT_MIN -255
#define OUTPUT_MAX  0
#define KP 40
#define KI 1.5
#define KD 0

#define TEMP_READ_INTERVAL 30000



int charge_current_setpoint = CHARGE_4A;         // 0-100 
int precharge_current_setpoint = PRECHARGE_TERM;         // 0-100 
int32_t frequency = 20000; //frequency (in Hz) for charging PWM


float charge_current, discharge_current, output19v_current, 
        output19v_voltage, bat_voltage, input_voltage;
        
double temp_in = 0, temp_bat = 0;

unsigned long lastmillis1Hz, lastmillis0_1Hz, button_pressed;

int last_button_state;

bool signal_led = 0;
unsigned long lastmillis_led;
bool led_state = 0;
int power_module_state = 0; // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged

bool output19_switch = 0;  // 0=off , 1=on
bool dc19_enable = 0;  // 1=off , 0=on
bool battery_discharge_switch = 1;  // 0=off , 1=on
bool battery_charge_switch = 1;  // 0=off , 1=on

bool online = 0;  // 0=offline 1=online
int battery_capacity = 0;
double battery_capacity_samples = 0;
int battery_capacity_samples_count = 0;
int charge_state = 0;  // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty


int fanSpeed = 0;
int fan_rpm = 0;
int fan_rpm_count = 0;

double outputVal;
double setPoint = 35.0;  // target temp for cooling 


void printData();
void getTemp();
void powerButton();
void getAdcData();
void signalLed();
void batteryCapacity();
void startUp();
void turnOff();
void standBy();
void turnOn();
void getOnLine();
void setChargeState();
void batteryEmpty();
void fanRpmInt();
void rosMsgUps();
void setChargeCurrent(const std_msgs::Int8& charge_current_msg);

ros::NodeHandle nh;

vitulus_ups::vitulus_ups vitulus_ups_msg;
ros::Publisher pub_ups("ups", &vitulus_ups_msg);

ros::Subscriber<std_msgs::Int8> sub_charge_current("charge_current", setChargeCurrent );

