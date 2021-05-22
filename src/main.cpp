#include <main.h>

OneWire oneWire1(ONE_WIRE_BUS_TEMP_SENSE_1);  
OneWire oneWire2(ONE_WIRE_BUS_TEMP_SENSE_2); 
DallasTemperature sensors1(&oneWire1);
DallasTemperature sensors2(&oneWire2);
Adafruit_ADS1115 ads;
AutoPID myPID(&temp_in, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);



void setup() {

  // Serial.begin(115200);
  pinMode(SIGNAL_LED, OUTPUT);
  digitalWrite(SIGNAL_LED, signal_led);
  
  pinMode(OUT19V_SWITCH, OUTPUT);
  digitalWrite(OUT19V_SWITCH, output19_switch);
  
  pinMode(OUT19V_DC_DC_EN, OUTPUT);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable);
  
  pinMode(BATTERY_CHARGE_SWITCH, OUTPUT);
  digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);
  
  pinMode(BATTERY_DISCHARGE_SWITCH, OUTPUT);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);

  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, fanSpeed);

  pinMode(FAN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_RPM), fanRpmInt, RISING);

  
  sensors1.begin();
  sensors2.begin();
  getTemp();

  ads.begin();
  
  pinMode(ISET2_PWM, OUTPUT);
  pinMode(ISET1_PWM, OUTPUT);
  InitTimersSafe(); 
  bool succ = SetPinFrequencySafe(ISET2_PWM, frequency);
  // Serial.println(succ);
  succ = SetPinFrequencySafe(ISET1_PWM, frequency);
  // Serial.println(succ);
  
  pwmWrite(ISET2_PWM, PRECHARGE_TERM);
  pwmWrite(ISET1_PWM, CHARGE_0A);


  pinMode(POWER_SWITCH_SENSE, INPUT);
  last_button_state = digitalRead(POWER_SWITCH_SENSE);
  startUp();

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(0.5, 0.5);
  //set PID update interval to 4000ms
  myPID.setTimeStep(TEMP_READ_INTERVAL);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_ups);
  nh.subscribe(sub_charge_current);


  lastmillis1Hz = millis();
  lastmillis0_1Hz = lastmillis1Hz;
}


void loop() {

  powerButton();
  getAdcData();
  batteryCapacity();
  getOnLine();
  setChargeState();
  signalLed();
  

  if ((millis() - lastmillis1Hz) >= 1000){
    lastmillis1Hz = millis();
    fan_rpm = (fan_rpm_count / 2) * 60;
    fan_rpm_count = 0;
    // printData();   
    rosMsgUps();
  }

  if ((millis() - lastmillis0_1Hz) >= TEMP_READ_INTERVAL){
    lastmillis0_1Hz = millis();
    getTemp();
    batteryEmpty();
    myPID.run(); //call every loop, updates automatically at certain time interval
    analogWrite(FAN_PWM, abs(outputVal));

  }
  nh.spinOnce();
   
}


void printData() {
  Serial.println("-----------------------------------------------------------");
  Serial.print("Charger current:  "); Serial.print(charge_current); Serial.println("A");
  Serial.print("Discharge current:  "); Serial.print(discharge_current); Serial.println("A");
  Serial.print("19V current:  "); Serial.print(output19v_current); Serial.println("A");
  Serial.print("19V voltage:  "); Serial.print(output19v_voltage); Serial.println("V");
  Serial.print("Battery voltage:  "); Serial.print(bat_voltage); Serial.println("V");
  Serial.print("Input voltage:  "); Serial.print(input_voltage); Serial.println("V");
  Serial.print("Internal temp:  "); Serial.print(temp_in); Serial.println("°C");
  Serial.print("Battery temp:  "); Serial.print(temp_bat); Serial.println("°C");
  Serial.print("Button state:  "); Serial.print(last_button_state); Serial.println("");
  Serial.print("Battery capacity:  "); Serial.print(battery_capacity); Serial.println("%");
  Serial.print("Power module state:  "); Serial.print(power_module_state); Serial.println("   0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged");
  Serial.print("Online:  "); Serial.print(online); Serial.println("   0=offline 1=online");
  Serial.print("Charge state:  "); Serial.print(charge_state); Serial.println("   0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty");
  Serial.print("fanPID output:  "); Serial.print(outputVal); Serial.println("   ");
  Serial.print("fan rpm:  "); Serial.print(fan_rpm); Serial.println("   ");
}


void getTemp() {
  sensors1.requestTemperatures();
  temp_in = sensors1.getTempCByIndex(0);
  sensors2.requestTemperatures();
  temp_bat = sensors2.getTempCByIndex(0);
}


void powerButton(){
  // Power button actions

  if (!digitalRead(POWER_SWITCH_SENSE)){
    button_pressed = millis();
    last_button_state = 0;
  }

  if (digitalRead(POWER_SWITCH_SENSE)){
    unsigned long button_time = millis() - button_pressed;

    if (last_button_state < 1) {
      // Serial.println("SWITCH PRESSED");
      last_button_state = 1;

    }

    if (button_time >= 1000 && button_time < 2000) {
      if (last_button_state < 2) {
        // Serial.println("SWITCH PRESSED 1s");
      }
      last_button_state = 2;
    }

    if (button_time >= 2000 && button_time < 4000) {
      if (last_button_state < 3) {
        // Serial.println("SWITCH PRESSED 2s");
        if (power_module_state == 2 || power_module_state == 3) {   // 0=startup  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
          turnOn();
        } else {
          standBy();
        }

      }
      last_button_state = 3;
    }

    if (button_time >= 4000 && button_time < 10000) {
      if (last_button_state < 4) {
        // Serial.println("SWITCH PRESSED 6s");
        turnOff();
      }
      last_button_state = 4;
    }

    if (button_time >= 10000 && button_time < 60000) {
      if (last_button_state < 5) {
        // Serial.println("SWITCH PRESSED 10s");
        startUp();
      }
      last_button_state = 5;
    }
  }
}


void getAdcData() {
  
  input_voltage  = (analogRead(VIN_ADC))*(30.0/1023.0);
  bat_voltage  = (analogRead(BATTERY_VSENSE_ADC))*(24.193/1023.0);
  charge_current = ((ads.computeVolts(ads.readADC_SingleEnded(0)))/(0.005*bat_voltage))-0.00; // -0.23
  if (!online) {
    charge_current = 0;
  }
  discharge_current = (ads.computeVolts(ads.readADC_SingleEnded(1)))/(0.005*bat_voltage)-0.00; // -0.05
  output19v_voltage = (ads.computeVolts(ads.readADC_SingleEnded(3)))*(24.0/5.0);
  if (output19v_voltage > 3){
    output19v_current = (ads.computeVolts(ads.readADC_SingleEnded(2)))/(0.005*output19v_voltage);
  } else {
    output19v_current = 0;
    output19v_voltage = 0;
  }
}


void signalLed() {
  if (power_module_state == 1) {   // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
    if (battery_capacity > BATT_LOW) {
      led_state = 1;
      digitalWrite(SIGNAL_LED, led_state);
      lastmillis_led = millis();
    }

    if (battery_capacity <= BATT_LOW && battery_capacity > BATT_EMPTY && (millis() - lastmillis_led) >= 500) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
      lastmillis_led = millis();
    }
    if (battery_capacity <= BATT_LOW && battery_capacity > BATT_EMPTY && led_state == 1 && (millis() - lastmillis_led) >= 50) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
    }

    if ( battery_capacity <= BATT_EMPTY && (millis() - lastmillis_led) >= 150) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
      lastmillis_led = millis();
    }
    if (battery_capacity <= BATT_EMPTY && led_state == 1 && (millis() - lastmillis_led) >= 50) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
    }
    

  }

  if (power_module_state == 2) {   // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
    if ((millis() - lastmillis_led) >= 1000) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
      lastmillis_led = millis();
    }
    if (led_state == 1 && (millis() - lastmillis_led) >= 50) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
    }
  }

  if (power_module_state == 3) {   // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
    if ((millis() - lastmillis_led) >= 6000) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
      lastmillis_led = millis();
    }
    if (led_state == 1 && power_module_state == 3 && (millis() - lastmillis_led) >= 50) {
      led_state = !led_state;
      digitalWrite(SIGNAL_LED, led_state);
    }
  }

}


void batteryCapacity() {

  battery_capacity_samples = battery_capacity_samples + (bat_voltage - BATT_EMPTY_VOLTAGE)/((BATT_FULL_VOLTAGE - BATT_EMPTY_VOLTAGE)/100.0);
  battery_capacity_samples_count ++ ;
  if (battery_capacity_samples_count >= SAMPLES) {
    battery_capacity =  battery_capacity_samples/battery_capacity_samples_count;
    if (battery_capacity > 100) {
      battery_capacity = 100;
    }
    battery_capacity_samples_count = 0;
    battery_capacity_samples = 0;
  }

}


void startUp() {
  // Turn on the arduino and sensors on start
  power_module_state = 0; // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
  output19_switch = 0;  // 0=off , 1=on
  dc19_enable = 1;  // 1=off , 0=on
  battery_discharge_switch = 1;  // 0=off , 1=on
  digitalWrite(OUT19V_SWITCH, output19_switch);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);
  standBy();
}


void turnOff(){
  // Turn off all module 
  output19_switch = 0;  // 0=off , 1=on
  dc19_enable = 1;  // 1=off , 0=on
  battery_charge_switch = 0;  // 0=off , 1=on
  battery_discharge_switch = 0;  // 0=off , 1=on
  digitalWrite(OUT19V_SWITCH, output19_switch);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable);
  digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);

}


void turnOn() {
  // Turn on the arduino and sensors on start
  power_module_state = 1; // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
  output19_switch = 1;  // 0=off , 1=on
  dc19_enable = 0;  // 1=off , 0=on
  battery_discharge_switch = 1;  // 0=off , 1=on
  digitalWrite(OUT19V_SWITCH, output19_switch);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);
  // battery_charge_switch = 0;  // 0=off , 1=on
  // digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);
}


void standBy() {
    // StandBy - outputs closed, charging possible
  power_module_state = 3; // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
  output19_switch = 0;  // 0=off , 1=on
  dc19_enable = 1;  // 1=off , 0=on
  battery_discharge_switch = 1;  // 0=off , 1=on
  digitalWrite(OUT19V_SWITCH, output19_switch);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);

}


void getOnLine() {
  if (input_voltage > ONLINE_MIN_VOLTAGE) {
    online = 1;  //0=offline 1=online
  } else {
    online = 0;  //0=offline 1=online
  }
}


void setChargeState() {
  if (online) {
    if (charge_state != 1) {   // if is online and not charged then start charging
      pwmWrite(ISET1_PWM, charge_current_setpoint);
      battery_charge_switch = 1;  // 0=off , 1=on
      digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);
      charge_state = 2; // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
      //pwmWrite(ISET2_PWM, PRECHARGE_TERM);
      if (power_module_state == 3) {
        power_module_state = 2;
      }
    }
    if (charge_state == 2) {
      if (battery_capacity == 100 && charge_current <= 0) {  // if is online, charging and charge current i bellow cut then stop charging
        pwmWrite(ISET1_PWM, charge_current_setpoint);
        battery_charge_switch = 0;  // 0=off , 1=on
        digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);
        charge_state = 1; // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
        if (power_module_state == 2) {
          power_module_state = 3;
        }
      }
    }

    
  } else {
    if (battery_capacity > BATT_LOW) {
      charge_state = 3; // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
    }
    if (battery_capacity < BATT_LOW && battery_capacity > BATT_EMPTY) {
      charge_state = 4; // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
    }
    if (battery_capacity < BATT_EMPTY) {
      charge_state = 5; // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
    }
  }
}

void batteryEmpty() {
  if (battery_capacity <= 0) {
    if (power_module_state == 3)  { // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
      turnOff();
    }
    if (power_module_state == 1)  { // 0=startup,  1=on, 2=stand-by_charging, 3=stand-by, 4=dicharged
      standBy();
    }
    
  }
}

void fanRpmInt() {
  fan_rpm_count ++;
}

void rosMsgUps()
{

    float bat_current;
    float in_current_avg;
    float pwr_bat = 0.0;
    float pwr_19v = 0.0;
    float pwr_in = 0.0;
    
    if (online){
      vitulus_ups_msg.ups_status = "online";
      

      if (charge_current > 0.0) {
        pwr_bat = bat_voltage * charge_current;
      }

      if (output19v_current > 0.0) {
        pwr_19v = output19v_voltage * output19v_current;
      }

      pwr_in = pwr_19v + pwr_bat;
      if (pwr_in > 0.0) {
        in_current_avg = pwr_in / input_voltage;
      }else {
        in_current_avg = 0.0;
      }
      

      
    }else{
      vitulus_ups_msg.ups_status = "offline";
      in_current_avg = 0.0;
    }
    // 0=init, 1=charged, 2=charging, 3=discharging, 4=low, 5=empty
    if (charge_state == 0){
      vitulus_ups_msg.battery_status = "Init";
    }
    if (charge_state == 1){
      vitulus_ups_msg.battery_status = "charged";
      bat_current = 0;
    }
    if (charge_state == 2){
      vitulus_ups_msg.battery_status = "charging";
      bat_current = charge_current;
    }
    if (charge_state == 3){
      vitulus_ups_msg.battery_status = "discharging";
      bat_current = discharge_current*-1;
    }
    if (charge_state == 4){
      vitulus_ups_msg.battery_status = "low";
      bat_current = discharge_current*-1;
    }
    if (charge_state == 5){
      vitulus_ups_msg.battery_status = "empty";
      bat_current = discharge_current*-1;
    }

   
    
    vitulus_ups_msg.driver = "Vitulus_UPS_v0.2";
    vitulus_ups_msg.battery_voltage = int (bat_voltage*1000);
    vitulus_ups_msg.battery_current = int (bat_current*1000);
    vitulus_ups_msg.input_voltage = int (input_voltage*1000);
    vitulus_ups_msg.input_current = int (in_current_avg*1000);
    vitulus_ups_msg.output_voltage = int (output19v_voltage*1000);
    vitulus_ups_msg.output_current = int (output19v_current*1000);
    vitulus_ups_msg.battery_capacity = int (battery_capacity);
    vitulus_ups_msg.cell1_voltage = int (charge_state);
    vitulus_ups_msg.cell2_voltage = int (pwr_in);
    vitulus_ups_msg.cell3_voltage = int (0);
    vitulus_ups_msg.cell4_voltage = int (0);
    vitulus_ups_msg.cell5_voltage = int (0);
    vitulus_ups_msg.cell6_voltage = int (0);
    pub_ups.publish(&vitulus_ups_msg);


    
}

void setChargeCurrent(const std_msgs::Int8& charge_current_msg) {

  if (charge_current_msg.data == 0){
    charge_current_setpoint = CHARGE_0A;
  }
  if (charge_current_msg.data == 1){
    charge_current_setpoint = CHARGE_1A;
  }
  if (charge_current_msg.data == 2){
    charge_current_setpoint = CHARGE_2A;
  }
  if (charge_current_msg.data == 4){
    charge_current_setpoint = CHARGE_4A;
  }
  if (charge_current_msg.data == 6){
    charge_current_setpoint = CHARGE_6A;
  }
  if (charge_current_msg.data == 10){
    charge_current_setpoint = CHARGE_FULL;
  }
}
