#include <main.h>
#include <tones.h>

OneWire oneWire1(ONE_WIRE_BUS_TEMP_SENSE_1);
DallasTemperature sensors1(&oneWire1);
Adafruit_ADS1115 ads;
AutoPID tempPID(&temp_in, &temp_setpoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID tempExtPID(&temp_ext, &temp_ext_setpoint, &outputExtTempVal, OUTPUT_EXT_MIN, OUTPUT_EXT_MAX, KP, KI, KD);
AVR_PWM* PWM_Buzzer;

/*############################################################################
#           SETUP
############################################################################*/
void setup()
{

  // Serial.begin(115200);
  // ROS node setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_ups);
  nh.advertise(pub_power_status);
  nh.advertise(pub_power_values);
  nh.subscribe(sub_set_precharge_current_running);
  nh.subscribe(sub_set_precharge_current_standby);
  nh.subscribe(sub_set_charge_current_running);
  nh.subscribe(sub_set_charge_current_standby);
  nh.subscribe(sub_set_charge_switch);
  nh.subscribe(sub_set_discharge_switch);
  nh.subscribe(sub_set_motor_switch);
  nh.subscribe(sub_set_bat_out_switch);
  nh.subscribe(sub_set_19v_out_switch);
  nh.subscribe(sub_set_temp_setpoint);
  nh.subscribe(sub_set_robot_sleep);
  nh.subscribe(sub_set_sleep_until_charged);
  nh.subscribe(sub_set_standby_timeout_discharg);
  nh.subscribe(sub_set_sleep_time);
  nh.subscribe(sub_set_sleep_wait_before_standby);
  nh.subscribe(sub_set_sleep_wait_charged_offset);
  nh.subscribe(sub_set_temp2_setpoint);

  // EEPROM First run check
  if (EEPROM.get(check_eeprom_addr, check_eeprom) != 1)
  { // first run
    check_eeprom = 1;
    EEPROM.put(check_eeprom_addr, check_eeprom);
    EEPROM.put(charge_current_setpoint_addr, charge_current_setpoint);
    EEPROM.put(precharge_current_setpoint_addr, precharge_current_setpoint);
    EEPROM.put(temp_setpoint_addr, temp_setpoint);
    EEPROM.put(standby_timeout_discharging_addr, standby_timeout_discharging);
    EEPROM.put(sleep_time_millis_addr, sleep_time_millis);
    EEPROM.put(sleep_wait_before_standby_millis_addr, sleep_wait_before_standby_millis);
    EEPROM.put(sleep_wait_charged_offset_millis_addr, sleep_wait_charged_offset_millis);
    EEPROM.put(charge_current_setpoint_running_addr, charge_current_setpoint_running);
    EEPROM.put(charge_current_setpoint_standby_addr, charge_current_setpoint_standby);
    EEPROM.put(precharge_current_setpoint_running_addr, precharge_current_setpoint_running);
    EEPROM.put(precharge_current_setpoint_standby_addr, precharge_current_setpoint_standby);
    EEPROM.put(temp_ext_setpoint_addr, temp_ext_setpoint);
  }
  else
  { // read stored values
    EEPROM.get(charge_current_setpoint_addr, charge_current_setpoint);
    EEPROM.get(precharge_current_setpoint_addr, precharge_current_setpoint);
    EEPROM.get(temp_setpoint_addr, temp_setpoint);
    EEPROM.get(standby_timeout_discharging_addr, standby_timeout_discharging);
    EEPROM.get(sleep_time_millis_addr, sleep_time_millis);
    EEPROM.get(sleep_wait_before_standby_millis_addr, sleep_wait_before_standby_millis);
    EEPROM.get(sleep_wait_charged_offset_millis_addr, sleep_wait_charged_offset_millis);
    EEPROM.get(sleep_wait_charged_offset_millis_addr, sleep_wait_charged_offset_millis);
    EEPROM.get(charge_current_setpoint_running_addr, charge_current_setpoint_running);
    EEPROM.get(charge_current_setpoint_standby_addr, charge_current_setpoint_standby);
    EEPROM.get(precharge_current_setpoint_running_addr, precharge_current_setpoint_running);
    EEPROM.get(precharge_current_setpoint_standby_addr, precharge_current_setpoint_standby);
    EEPROM.get(temp_ext_setpoint_addr, temp_ext_setpoint);
  }

  pinMode(ISET2_PWM, OUTPUT);
  pinMode(ISET1_PWM, OUTPUT);
  InitTimersSafe();
  SetPinFrequencySafe(ISET2_PWM, frequency);
  SetPinFrequencySafe(ISET1_PWM, frequency);

  PWM_Buzzer = new AVR_PWM(BUZZER, T00, VNO);
  if (PWM_Buzzer)
  {
    PWM_Buzzer->setPWM();
  }


  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, fanSpeed);

  pinMode(FAN2_PWM, OUTPUT);
  analogWrite(FAN2_PWM, fan2Speed);

  pinMode(TEMP_PWR, OUTPUT);

  pinMode(POWER_SWITCH_SENSE, INPUT);
  pinMode(PG, INPUT);

  pinMode(BATTERY_CHARGE_SWITCH, OUTPUT);
  digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);

  pinMode(BATTERY_DISCHARGE_SWITCH, OUTPUT);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);

  pinMode(OUT19V_SWITCH, OUTPUT);
  digitalWrite(OUT19V_SWITCH, output19_switch);

  pinMode(SIGNAL_LED, OUTPUT);
  digitalWrite(SIGNAL_LED, LOW);

  pinMode(OUT19V_DC_DC_EN, OUTPUT);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable); // reversed

  pinMode(POWER_EN, OUTPUT);
  digitalWrite(POWER_EN, power_enable_switch);

  pinMode(BATT_OUTPUT_SWITCH, OUTPUT);
  digitalWrite(BATT_OUTPUT_SWITCH, battery_output_switch);

  pinMode(MOTORS_OUTPUT_SWITCH, OUTPUT);
  digitalWrite(MOTORS_OUTPUT_SWITCH, motors_output_switch);

  pinMode(FAN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_RPM), fanRpmInt, RISING);

  pinMode(FAN2_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN2_RPM), fan2RpmInt, RISING);

  // if temperature is more than x degrees below or above setpoint, OUTPUT will be set to min or max respectively
  tempPID.setBangBang(1.0, 1.0);
  tempExtPID.setBangBang(3.0, 3.0);
  // PID update interval
  tempPID.setTimeStep(TEMP_INT_PID_STEP);
  tempExtPID.setTimeStep(TEMP_EXT_PID_STEP);

  sensors1.begin();
  ads.begin();

  lastmillis_1Hz = millis();
  lastmillis_20Hz = millis();
  lastmillis_2Hz = millis();
  lastmillis_led = millis();
  lastmillis_pwr_button = millis();
  lastmillis_timeout = millis();
  sleep_time_timer = millis();
  millis_play_note = millis();

  melody_playing = false;

  // init qeueue for melodies and set beep after booted to play.
  melody_buffer[0] = beep.melody_name;
  melody_buffer[1] = 0;
  melody_buffer[2] = 0;
  melody_buffer[3] = 0;


}

/*############################################################################
#           ############   MAIN LOOP   ###########
############################################################################*/
void loop()
{
  get_ADS();
  get_bq24610();
  get_voltage();
  get_ntc();
  // get_ds18b20();
  get_input_status();
  get_charger_status();
  batteryCapacity();
  setChargingSwitch();
  overDischargeOff();
  standbyTimeout();
  set_charge_current();
  check_sleep();

  tempPID.run();  // Run PID computation
  tempExtPID.run();   // Run PID computation
  analogWrite(FAN_PWM, int(abs(outputVal))); // Write fan speed from PID.
  analogWrite(FAN2_PWM, int(abs(outputExtTempVal))); // Write fan speed from PID.

  // 1Hz timer
  if ((millis() - lastmillis_1Hz) >= 1000)
  {
    lastmillis_1Hz = millis();
    fan_rpm = (fan_rpm_count / 2) * 60;
    fan_rpm_count = 0;
    fan2_rpm = (fan2_rpm_count / 2) * 60;
    fan2_rpm_count = 0;
    // printData();
    rosMsgUps();
  }
  // 20Hz timer
  if ((millis() - lastmillis_20Hz) >= 50)
  {
    lastmillis_20Hz = millis();
    rosMsgPowerValues();
  }
  // 500 miliseconds timer
  if ((millis() - lastmillis_2Hz) >= 500)
  {
    lastmillis_2Hz = millis();
    rosMsgPowerStatus();
  }
  // led timer
  if ((millis() - lastmillis_led) >= lastmillis_led_interval)
  {
    lastmillis_led = millis();
    set_led();
  }

  // play_tones();
  play_melody();

  get_pwrswitch(); // Last before the pin witing to override all if power button event

  /// For testing only ////////////////////////////////
  // motors_output_switch = 1;
  // power_enable_switch = 0;  // 0=off , 1=on
  // battery_discharge_switch = 0;
  // battery_charge_switch = 1;
  /////////////////////////////////////////////////////

  digitalWrite(POWER_EN, power_enable_switch);
  digitalWrite(BATTERY_CHARGE_SWITCH, battery_charge_switch);
  digitalWrite(BATTERY_DISCHARGE_SWITCH, battery_discharge_switch);
  digitalWrite(OUT19V_DC_DC_EN, dc19_enable); // reversed
  digitalWrite(OUT19V_SWITCH, output19_switch);
  digitalWrite(OUT19V_SWITCH, output19_switch);
  digitalWrite(BATT_OUTPUT_SWITCH, battery_output_switch);
  digitalWrite(MOTORS_OUTPUT_SWITCH, motors_output_switch);

  // printData();
  nh.spinOnce();
}

/*############################################################################
#           Add melody to play to buffer
############################################################################*/
void add_melody(int melody_to_add){
  for(int i=0; i < MELODY_BUFFER; i++){
    if (!melody_buffer[i]){
      melody_buffer[i] = melody_to_add;
      break;
    }
  }
}

/*############################################################################
#           Return melody to play from buffer, if any or (0)
############################################################################*/
int get_melody(){
  int return_melody = 0;
  if (melody_buffer[0]){
    return_melody = melody_buffer[0];
  }
  return return_melody;
}

/*############################################################################
#           Return melody structure by id
############################################################################*/
melodya id_get_melody(int id_m){
  if (id_m == 101){
    return melody_online;
  }
  if (id_m == 102){
    return melody_offline;
  }
  if (id_m == 103){
    return melody_charging;
  }
  if (id_m == 104){
    return melody_charged;
  }
  if (id_m == 1){
    return beep;
  }
  if (id_m == 2){
    return beep_double;
  }
  if (id_m == 3){
    return alarm;
  }
  if (id_m == 4){
    return horn;
  }
  if (id_m == 5){
    return short_beep;
  }
}

/*############################################################################
#           Shift buffer of melodies
############################################################################*/
void shift_buffer(){
  melody_buffer[0] = melody_buffer[1];
  melody_buffer[1] = melody_buffer[2];
  melody_buffer[2] = melody_buffer[3];
}

/*############################################################################
#           Play melody
############################################################################*/
void play_melody()
{
  if (!melody_playing){
    int melody_tmp = get_melody();
    
    if (melody_tmp){
      melody_playing = true;
      melody = id_get_melody(melody_tmp);
      shift_buffer();
      // nh.loginfo("********** play_melody *************");
      char buffer[80];
      // sprintf(buffer, "DEBUG play_melody nr.: %d ", melody.melody_name);
      nh.loginfo(buffer);
    }
  }

  if (melody_playing){
    if (melody.id == 100){
      millis_play_note = millis();
      melody.id = 0;
      // PWM_Buzzer->setPWM(BUZZER, melody.note[melody.id], VLO);
      PWM_Buzzer->setPWM(BUZZER, melody.note[melody.id], melody.note_volume[melody.id]);
    }
    // int last_melody_id = melody.id;
    if ((millis()- millis_play_note) >= melody.note_length[melody.id]){  // If note duration expired
      melody.id ++;  // Move to next note
      millis_play_note = millis();
      // PWM_Buzzer->setPWM(BUZZER, melody.note[melody.id], VLO);
      PWM_Buzzer->setPWM(BUZZER, melody.note[melody.id], melody.note_volume[melody.id]);
      if (melody.id > (melody.note_count-1)){  // If melody ends
        melody.id = 100;         // Go to begin
        melody_playing = false; 
        PWM_Buzzer->setPWM(BUZZER, melody.note[melody.id], VNO);       
      }
    }

  }
  
}

/*############################################################################
#           Compute the amps from adc voltage
############################################################################*/
float get_amps(float val, float mv_A)
{

  float output = 0.00;
  if (val >= 2.51)
  {
    output = (val - 2.50) * 1000.0 / mv_A;
  }
  if (val <= 2.49)
  {
    output = (-2.50 + val) * 1000.0 / mv_A;
  }
  return output;
}

/*############################################################################
#           Read current values from ADS1115
############################################################################*/
void get_ADS()
{
  float adc3 = ads.computeVolts(ads.readADC_SingleEnded(3));
  adc3 = (get_amps((adc3 + 0.00), 100)) * -1;
  if (adc3 <= 0.0){
    input_i = 0.0;
  }else{
    input_i = adc3;
  } 
  
  float adc0 = ads.computeVolts(ads.readADC_SingleEnded(0));
  adc0 = (get_amps((adc0 - 0.01), 100));
  if (adc0 <= 0.0){
    charge_i = 0.0;
  }else{
    charge_i = adc0;
  }

  float adc1 = ads.computeVolts(ads.readADC_SingleEnded(1));
  adc1 = (get_amps((adc1 + 0.00), 66));
  if (adc1 <= 0.0){
    discharge_i = 0.0;
  }else{
    discharge_i = adc1;
  }

  float adc2 = ads.computeVolts(ads.readADC_SingleEnded(2));
  adc2 = (get_amps((adc2 - 0.01), 100)) * -1;
  if (adc2 <= 0.0){
    out19_i = 0.0;
  }else{
    out19_i = adc2;
  }
  
}

/*############################################################################
#              Read outputs of charger IC - same as leds on board
############################################################################*/
void get_bq24610()
{
  stat_1 = !digitalRead(STAT_1);
  stat_2 = !digitalRead(STAT_2);
  pg = !digitalRead(PG);
}

/*############################################################################
#               Read input, battery, 19V output voltages
############################################################################*/
void get_voltage()
{
  bat_v = (analogRead(BATTERY_VSENSE_ADC) * (30000.0 / 1023.0)) / 1000;
  input_v = (analogRead(VIN_ADC) * (30000.0 / 1023.0)) / 1000;
  out19_v = (analogRead(V19_VSENSE_ADC) * (24000.0 / 1023.0)) / 1000;
}

/*############################################################################
#              Temperature readings from NTC thermistors
############################################################################*/
void get_ntc()
{
  
  digitalWrite(TEMP_PWR, HIGH);
  tmp_in_buff = tmp_in_buff + analogRead(TEMP_NTC);
  tmp_ext_buff = tmp_ext_buff + analogRead(TEMP_NTC_EXT);
  digitalWrite(TEMP_PWR, LOW);
  tmp_buff_count ++;
  calculate_ntc();
  
}

/*############################################################################
#              Temperature calculation
############################################################################*/
void calculate_ntc()
{
  if(tmp_buff_count >= TEMP_SAMPLES)
  {
    double tmp_in;
    tmp_in = tmp_in_buff / TEMP_SAMPLES;
    // Calculate NTC resistance internal
    tmp_in = 1023 / tmp_in - 1;
    tmp_in = Rref / tmp_in;
    tmp_in = tmp_in / nominal_resistance;    // (R/Ro)
    tmp_in = log(tmp_in);                      // ln(R/Ro)
    tmp_in /= beta;                                 // 1/B * ln(R/Ro)
    tmp_in += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
    tmp_in = 1.0 / tmp_in;                     // Invert
    temp_in = tmp_in - 273.15;                     // convert absolute temp to C
    // Calculate NTC resistance external
    double tmp_ext;
    tmp_ext = tmp_ext_buff / TEMP_SAMPLES;
    tmp_ext = 1023 / tmp_ext - 1;
    tmp_ext = Rref / tmp_ext;
    tmp_ext = tmp_ext / nominal_resistance;    // (R/Ro)
    tmp_ext = log(tmp_ext);                      // ln(R/Ro)
    tmp_ext /= beta;                                 // 1/B * ln(R/Ro)
    tmp_ext += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
    tmp_ext = 1.0 / tmp_ext;                     // Invert
    temp_ext = tmp_ext - 273.15;                     // convert absolute temp to C

    tmp_buff_count = 0;
    tmp_in_buff = 0;
    tmp_ext_buff = 0;

  }
  
}

/*############################################################################
#           ds18b20 temperature sensor readings
############################################################################*/
void get_ds18b20()
{
  sensors1.requestTemperatures();
  ds_temp = sensors1.getTempCByIndex(0);
}

/*############################################################################
#             Control power button changes.
############################################################################*/
void get_pwrswitch()
{
  if (digitalRead(POWER_SWITCH_SENSE)) // if button is pressed
  {
    if (pwr_switch)  // still pressed
    {                                                 
      if ((millis() - lastmillis_pwr_button) >= 4000) // shutdown
      {
        if ((millis() - lastmillis_pwr_button) < 4040) // shutdown beep
        {
          add_melody(beep_double.melody_name);
        }
        nh.loginfo("Shutdown by power button.");
        set_shutdown();
      }
      if ((millis() - lastmillis_pwr_button) >= 2000 && (millis() - lastmillis_pwr_button) < 2040) // power on time (beep)
      {
        add_melody(beep.melody_name);
      }
      // MELODY test ToDo: comment **********************************************************
      // if ((millis() - lastmillis_pwr_button) >= 500 && (millis() - lastmillis_pwr_button) < 550) // power on time (beep)
      // {
      //   add_melody(alarm.melody_name);
      // }
      pwr_switch = 1;
    }
    else // changed state to pressed
    {
      lastmillis_pwr_button = millis();
      pwr_switch = 1;
    }
  }
  else // if button is released.
  {
    if (!pwr_switch)
    { // still released
      pwr_switch = 0;
    }
    else // changed state to released
    {
      pwr_switch = 0;
      if ((millis() - lastmillis_pwr_button) > 0 && (millis() - lastmillis_pwr_button) < 2000)
      {
        if (power_module_status == STARTUP) // poweron to standby on startup
        {
          set_standby();
          nh.loginfo("Stanby by power button.");
        }
      }
      if ((millis() - lastmillis_pwr_button) >= 2000 && (millis() - lastmillis_pwr_button) < 4000)
      {
        if (power_module_status == STARTUP || power_module_status == STANDBY)
        {
          // poweron robot
          set_robot_poweron();
          nh.loginfo("Power on by power button.");
        }
        else
        {
          // poweroff robot to stand-by
          set_standby();
          nh.loginfo("Stanby by power button.");
        }
      }
    }
  }
}

/*############################################################################
#             Interrupt from fans encoders.
############################################################################*/
void fanRpmInt()
{
  fan_rpm_count++;
}

void fan2RpmInt()
{
  fan2_rpm_count++;
}

/*############################################################################
#             Check the power supply status.
############################################################################*/
void get_input_status()
{
  // if buffer full
  if (pg_counter >= 25)
  {
    pg_buffer = pg_buffer / pg_counter;

    if (pg_buffer == 1.0)
    {
      input_status = ONLINE;
    }
    if (input_v > bat_v)
    {
      if (pg_buffer < 1.0)
      {
        input_status = FAIL;
      }
    }
    if (pg_buffer == 0)
    {
      input_status = OFFLINE;
    }
    pg_counter = 0;
    pg_buffer = 0;
  }
  // Power suply status events. (On status change)
  if (last_input_status != input_status)
  { 
    if (input_status == OFFLINE)
    {
      sleep_charged_timer = millis();
      nh.loginfo("Power supply disconected.");
      add_melody(short_beep.melody_name);
    }
    if (input_status == ONLINE)
    {
      nh.loginfo("Power supply conected");
      add_melody(short_beep.melody_name);
    }
    if (input_status == FAIL)
    {
      nh.logwarn("Power supply failing.");
    }
  }
  last_input_status = input_status;

  // write pg state to buffer
  if (pg)
  {
    pg_buffer += 1;
  }
  else
  {
    pg_buffer += 0;
  }
  pg_counter += 1;
}

/*############################################################################
#             Check the charger status.
############################################################################*/
void get_charger_status()
{
  if (input_status == 1)
  {
    if (stat_1 == 1)
    {
      charger_status = CHARGING;
    }
    if (stat_2 == 1)
    {
      charger_status = CHARGED;
    }
  }
  else
  {
    charger_status = DISCHARGING;
  }
  // Charger status events. (On status change)
  if (last_charger_status != charger_status)
  { 
    if (charger_status == CHARGED)
    {
      add_melody(melody_charged.melody_name);
      sleep_charged_timer = millis();
      nh.loginfo("Battery charged");
    }
    if (charger_status == DISCHARGING)
    {
      nh.loginfo("Battery start discharging");
    }
    if (charger_status == CHARGING)
    {
      nh.loginfo("Battery start charging");
      add_melody(melody_charging.melody_name);
    }
  }
  last_charger_status = charger_status;
}

/*############################################################################
#             Manage the power button LED in dependence on charger status.
############################################################################*/
void set_led()
{
  if (led_state)
  {

    if (charger_status == 0)
    { //  discharging
      led_state = false;
      lastmillis_led_interval = LED_DISCHARGING_OFF;
    }
    if (charger_status == 1)
    { //  charged
      lastmillis_led_interval = LED_CHARGED_OFF;
    }
    if (charger_status == 2)
    { //  charging
      led_state = false;
      lastmillis_led_interval = LED_CHARGING_OFF;
    }
  }
  else
  {
    led_state = true;
    if (charger_status == 0)
    { //  discharging
      lastmillis_led_interval = LED_DISCHARGING_ON;
    }
    if (charger_status == 1)
    { //  charged
      lastmillis_led_interval = LED_CHARGED_ON;
    }
    if (charger_status == 2)
    { //  charging
      lastmillis_led_interval = LED_CHARGING_ON;
    }
  }
  digitalWrite(SIGNAL_LED, led_state);
}

/*############################################################################
#               Battery capacity computing on samples of voltage. Better something than nothing:)
############################################################################*/
void batteryCapacity()
{
  int last_batt_capacity = battery_capacity;
  if (battery_capacity == -100) // if init
  {
    battery_capacity = (bat_v - BATT_EMPTY_VOLTAGE) / ((BATT_FULL_VOLTAGE - BATT_EMPTY_VOLTAGE) / 100.0);
  }
  battery_capacity_samples = battery_capacity_samples + (bat_v - BATT_EMPTY_VOLTAGE) / ((BATT_FULL_VOLTAGE - BATT_EMPTY_VOLTAGE) / 100.0);
  battery_capacity_samples_count++;
  if (battery_capacity_samples_count >= BAT_SAMPLES)
  {
    battery_capacity = battery_capacity_samples / battery_capacity_samples_count;
    battery_capacity_samples_count = 0;
    battery_capacity_samples = 0;
  }
  if (battery_capacity > 100)
  {
    battery_capacity = 100;
  }
  if (battery_capacity < 0)
  {
    battery_capacity = 0;
  }
  if (charger_status == DISCHARGING && battery_capacity < 5 && battery_capacity != last_batt_capacity){
    add_melody(alarm.melody_name);
  }
}

/*############################################################################
#               Turn off the charge mosfet if no power supply connected.
############################################################################*/
void setChargingSwitch()
{
  if (input_status == OFFLINE)
  {
    battery_charge_switch = 0; // 0=off , 1=on
  }
  else
  {
    battery_charge_switch = 1; // 0=off , 1=on
  }
}

/*############################################################################
#               Set the switches variables to robot power on.
############################################################################*/
void set_robot_poweron()
{
  // poweron robot
  output19_switch = 1;           // 0=off , 1=on
  dc19_enable = 0;               // 1=off , 0=on
  battery_discharge_switch = 1;  // 0=off , 1=on
  power_enable_switch = 1;       // 0=off , 1=on
  battery_output_switch = 0;     // 0=off , 1=on
  motors_output_switch = 0;      // 0=off , 1=on
  power_module_status = RUNNING; // 0=startup,  1=on, 2=stand-by
}

/*############################################################################
#               Set the switches variables to standby.
############################################################################*/
void set_standby()
{
  // stand-by
  output19_switch = 0;           // 0=off , 1=on
  dc19_enable = 1;               // 1=off , 0=on
  battery_discharge_switch = 1;  // 0=off , 1=on
  power_enable_switch = 1;       // 0=off , 1=on
  battery_output_switch = 0;     // 0=off , 1=on
  motors_output_switch = 0;      // 0=off , 1=on
  power_module_status = STANDBY; // 0=startup,  1=on, 2=stand-by
}

/*############################################################################
#               Set the switches variables to shutdown.
############################################################################*/
void set_shutdown()
{
  // shutdown
  output19_switch = 0;           // 0=off , 1=on
  dc19_enable = 1;               // 1=off , 0=on
  battery_discharge_switch = 0;  // 0=off , 1=on
  battery_charge_switch = 0;     // 0=off , 1=on
  power_enable_switch = 0;       // 0=off , 1=on
  battery_output_switch = 0;     // 0=off , 1=on
  motors_output_switch = 0;      // 0=off , 1=on
  power_module_status = STARTUP; // 0=startup,  1=on, 2=stand-by
}

/*############################################################################
#               Emergency shutdown on no power supply
#               and overdischarged battery.
############################################################################*/
void overDischargeOff()
{
  if (bat_v < 17.8 && input_v < bat_v)
  {
    nh.loginfo("Overdischarge, shutting down.");
    set_shutdown();
  }
}

/*############################################################################
#               Shutdown after timeout if dicharging,
#               not operate and power offline
############################################################################*/
void standbyTimeout()
{
  if (!standby_timeout_running)  // if counter not started
  { 
    if (charger_status == 0)  // 0=discharging, 1=charged, 2=charging
    { 
      if (input_status != 1)  // 0=offline, 1=online, 2=fail
      { 
        if (output19_switch == 0)  // robot not operate   0=off , 1=on
        { 
          standby_timeout_running = 1;  
          lastmillis_timeout = millis();  // start countdown
          nh.loginfo("Standby timeout started.");
        }
      }
    }
  }
  if (standby_timeout_running)
  { 
    if (charger_status != 0 || input_status == 1 || output19_switch == 1)  // cancel timeout if conditions changed
    {
      standby_timeout_running = 0; 
      nh.loginfo("Standby timeout canceled.");
    }
  }
  if (standby_timeout_running)
  { // shutdown when timeout exceed and not in sleep (battery saving)
    if ((millis() - lastmillis_timeout) >= standby_timeout_discharging)
    {
      if (!sleep_status && !sleep_charged_status)
      { 
        standby_timeout_running = false;
        nh.loginfo("Standby timeout exceeded. Shutting down.");
        set_shutdown();
      }
    }
  }
}

/*############################################################################
#               Set precharge and charge currents.
############################################################################*/
void set_charge_current()
{ 
  // Set charge current by module status. (RUNNING, STANDBY)
  if (power_module_status == RUNNING){
    charge_current_setpoint = charge_current_setpoint_running;
    precharge_current_setpoint = precharge_current_setpoint_running;
  }
  if (power_module_status == STANDBY){
    charge_current_setpoint = charge_current_setpoint_standby;
    precharge_current_setpoint = precharge_current_setpoint_standby;
  }
  // Check the limits for charge current. DO NOT CHANGE! 100 = 10A
  if (charge_current_setpoint < 0 && charge_current_setpoint > 81)
  {
    charge_current_setpoint = 0;
  }
  // Check the limits for precharge current and cut off current. DO NOT CHANGE! 100 = 2A
  if (precharge_current_setpoint < PRECHARGE_TERM_MIN && precharge_current_setpoint > PRECHARGE_TERM_MAX)
  {
    precharge_current_setpoint = PRECHARGE_TERM;
  }
  // PWM_I1SET->setPWM(ISET1_PWM, frequency, float(charge_current_setpoint)/2.0f);
  // PWM_I2SET->setPWM(ISET2_PWM, frequency, float(precharge_current_setpoint)/2.0f);
  // PWM_I1SET->setPWM(ISET1_PWM, frequency, 1.0f);
  // PWM_I2SET->setPWM(ISET2_PWM, frequency, 40.0f);
  pwmWrite(ISET1_PWM, charge_current_setpoint);
  pwmWrite(ISET2_PWM, precharge_current_setpoint);
}

/*############################################################################
#               Manage sleep. If conditions are met,
#               set robot to stanby, or turn on.
############################################################################*/
void check_sleep()
{
  // Sleep for time interval.
  if (sleep_status)
  {
    // If timeout for the robot's self shutdown exceeded. Go to stanby and sleep.
    if ((millis() - sleep_wait_before_standby_timer) >= sleep_wait_before_standby_millis && (millis() - sleep_time_timer) <= sleep_wait_before_standby_millis + 300)
    {
      if (power_module_status == STANDBY){
        set_standby();
        nh.loginfo("Sleep for time interval go to standby.");
      }
      
    }
    // If robot's sleep interval exceeded. Wake up.
    if ((millis() - sleep_time_timer) >= sleep_time_millis)
    {
      sleep_status = false;
      set_robot_poweron();
      nh.loginfo("Wake up from sleep. (time interval)");
    }
  }
  // Sleep while charging. Wake up if charged and offset exceed.
  if (sleep_charged_status)
  {
    // If timeout for the robot's self shutdown exceeded. Go to stanby and sleep.
    if ((millis() - sleep_wait_before_standby_timer) >= sleep_wait_before_standby_millis && (millis() - sleep_time_timer) <= sleep_wait_before_standby_millis + 300)
    { 
      if (power_module_status == STANDBY){
        set_standby();
        nh.loginfo("Sleep for until charged go to standby.");
      }
    }
    if (charger_status == CHARGED)
    {
      if ((millis() - sleep_charged_timer) >= sleep_wait_charged_offset_millis)
      {
        set_robot_poweron();
        sleep_charged_status = false;
        nh.loginfo("Wake up from sleep. (until charged)");
      }
    }
  }
}

/*############################################################################
#               PRINT SOME RUNNING DATA TO SERIAL
#               (DISABLE ROS AND BEGIN SERIAL)
############################################################################*/
void printData()
{
  // input
  Serial.print("In: ");
  Serial.print(input_v);
  Serial.print("V ");
  Serial.print(input_i);
  Serial.print("A ");
  // battery
  Serial.print("Bat: ");
  Serial.print(bat_v);
  Serial.print("V ");
  Serial.print(charge_i);
  Serial.print("A ");
  Serial.print(discharge_i);
  Serial.print("A ");
  Serial.print(battery_capacity);
  Serial.print("% ");
  // 19V output
  Serial.print("Out19: ");
  Serial.print(out19_v);
  Serial.print("V ");
  Serial.print(out19_i);
  Serial.print("A ");
  // ntc temp
  Serial.print("Temp: ");
  Serial.print(temp_in);
  Serial.print("°C ");
  // ntc temp ext
  Serial.print("TempExt: ");
  Serial.print(temp_ext);
  Serial.print("°C ");
  // ds temp
  Serial.print("Temp_DS: ");
  Serial.print(ds_temp);
  Serial.print("°C ");
  // Charging STAT1
  Serial.print("STAT1: ");
  Serial.print(stat_1);
  Serial.print(" ");
  // Charged STAT2
  Serial.print("STAT2: ");
  Serial.print(stat_2);
  Serial.print(" ");
  // POWER GOOD
  Serial.print("PG: ");
  Serial.print(pg);
  Serial.print(" ");
  // POWER SWITCH
  Serial.print("PWR_SW: ");
  Serial.print(pwr_switch);
  Serial.print(" ");
  // Power line status
  Serial.print("PowerIn: ");
  if (input_status == 1)
  {
    Serial.print("ONLINE");
  }
  if (input_status == 0)
  {
    Serial.print("OFFLINE");
  }
  if (input_status == 2)
  {
    Serial.print("FAIL");
  }
  Serial.print(" ");
  // Charger status
  Serial.print("Charger: ");
  if (charger_status == 1)
  {
    Serial.print("CHARGED");
  }
  if (charger_status == 0)
  {
    Serial.print("DISCHARGING");
  }
  if (charger_status == 2)
  {
    Serial.print("CHARGING");
  }
  Serial.print(" ");
  // FAN RPM
  Serial.print("Fan: ");
  Serial.print(fan_rpm);
  Serial.print("RPM ");
  Serial.print("\t");
  // FAN2 RPM
  Serial.print("Fan2: ");
  Serial.print(fan2_rpm);
  Serial.print("RPM ");
  Serial.println("");
}

/*############################################################################
#               Old status message DEPRECATED
############################################################################*/
void rosMsgUps()
{
  if (input_status == ONLINE)
  {
    vitulus_ups_msg.ups_status = "online";
  }
  if (input_status == OFFLINE)
  {
    vitulus_ups_msg.ups_status = "offline";
  }
  if (input_status == FAIL)
  {
    vitulus_ups_msg.ups_status = "fail";
  }

  if (charger_status == DISCHARGING)
  {
    vitulus_ups_msg.battery_status = "discharging";
  }
  if (charger_status == CHARGING)
  {
    vitulus_ups_msg.battery_status = "charging";
  }
  if (charger_status == CHARGED)
  {
    vitulus_ups_msg.battery_status = "charged";
  }
  vitulus_ups_msg.driver = "Vitulus_UPS_v3.1";
  vitulus_ups_msg.battery_voltage = int(bat_v * 1000);
  vitulus_ups_msg.battery_current = int(((discharge_i * -1) + charge_i) * 1000);
  vitulus_ups_msg.input_voltage = int(input_v * 1000);
  vitulus_ups_msg.input_current = int(input_i * 1000);
  vitulus_ups_msg.output_voltage = int(out19_v * 1000);
  vitulus_ups_msg.output_current = int(out19_i * 1000);
  vitulus_ups_msg.battery_capacity = int(battery_capacity);
  vitulus_ups_msg.cell1_voltage = int(charge_i * 1000);
  vitulus_ups_msg.cell2_voltage = int(discharge_i * 1000);
  vitulus_ups_msg.cell3_voltage = int(power_module_status);
  vitulus_ups_msg.cell4_voltage = int(charge_current_setpoint);
  vitulus_ups_msg.cell5_voltage = int(0);
  vitulus_ups_msg.cell6_voltage = int(0);
  pub_ups.publish(&vitulus_ups_msg);
}

/*############################################################################
#               Full status message
############################################################################*/
void rosMsgPowerStatus()
{
  power_status_msg.header.stamp = nh.now();
  power_status_msg.version = "Vitulus_PM_v3.1";
  if (charger_status == 0)
  {
    power_status_msg.charger_status = "DISCHARGING";
  }
  if (charger_status == 1)
  {
    power_status_msg.charger_status = "CHARGED";
  }
  if (charger_status == 2)
  {
    power_status_msg.charger_status = "CHARGING";
  }

  if (input_status == 0)
  {
    power_status_msg.supply_status = "OFFLINE";
  }
  if (input_status == 1)
  {
    power_status_msg.supply_status = "ONLINE";
  }
  if (input_status == 2)
  {
    power_status_msg.supply_status = "FAIL";
  }

  if (power_module_status == STARTUP)
  {
    power_status_msg.module_status = "STARTUP";
  }
  if (power_module_status == RUNNING)
  {
    power_status_msg.module_status = "RUNNING";
  }
  if (power_module_status == STANDBY)
  {
    power_status_msg.module_status = "STANDBY";
  }

  power_status_msg.input_voltage = input_v;
  power_status_msg.input_current = input_i;
  power_status_msg.input_power = input_v * input_i;
  power_status_msg.battery_voltage = bat_v;
  power_status_msg.battery_current = (discharge_i * -1) + charge_i;
  power_status_msg.battery_power = bat_v * ((discharge_i * -1) + charge_i);
  power_status_msg.battery_charge_current = charge_i;
  power_status_msg.battery_charge_power = bat_v * charge_i;
  power_status_msg.battery_discharge_current = discharge_i;
  power_status_msg.battery_discharge_power = bat_v * discharge_i;
  power_status_msg.battery_capacity = battery_capacity;
  power_status_msg.out19_voltage = out19_v;
  power_status_msg.out19_current = out19_i;
  power_status_msg.out19_power = out19_v * out19_i;
  power_status_msg.temp = temp_in;
  power_status_msg.temp2 = temp_ext;
  power_status_msg.fan_rpm = fan_rpm;
  power_status_msg.fan2_rpm = fan2_rpm;
  power_status_msg.charge_switch = battery_charge_switch;
  power_status_msg.discharge_switch = battery_discharge_switch;
  power_status_msg.bat_out_switch = battery_output_switch;
  power_status_msg.motor_out_switch = motors_output_switch;
  power_status_msg.out19v_switch = output19_switch;
  power_status_msg.enable19v_switch = !dc19_enable;
  power_status_msg.charge_current_setpoint_run = int(float(charge_current_setpoint_running) / 0.01063 / 1.23);
  power_status_msg.charge_current_setpoint_standby = int(float(charge_current_setpoint_standby) / 0.01063 / 1.23);
  power_status_msg.precharge_current_setpoint_run = int(float(precharge_current_setpoint_running) / 0.05355) - 300;
  power_status_msg.precharge_current_setpoint_standby = int(float(precharge_current_setpoint_standby) / 0.05355) - 300;
  power_status_msg.temp_setpoint = temp_setpoint;
  power_status_msg.temp2_setpoint = temp_ext_setpoint;
  power_status_msg.standby_timeout = standby_timeout_discharging;
  power_status_msg.sleep_status = sleep_status;
  power_status_msg.sleep_charged_status = sleep_charged_status;
  power_status_msg.sleep_wait_standby = sleep_wait_before_standby_millis;
  power_status_msg.sleep_time_interval = sleep_time_millis;
  power_status_msg.sleep_wait_charged = sleep_wait_charged_offset_millis;
  char buffer[80];
  sprintf(buffer, "DEBUG INFO: %f int: %d", outputExtTempVal, int(abs(outputExtTempVal)));
  power_status_msg.debug_info = buffer;
  

  pub_power_status.publish(&power_status_msg);
}

/*############################################################################
#               Message contains fast changing values e.g. voltage, current, etc.
############################################################################*/
void rosMsgPowerValues()
{
  power_values_msg.header.stamp = nh.now();
  power_values_msg.input_voltage = input_v;
  power_values_msg.input_current = input_i;
  power_values_msg.input_power = input_v * input_i;
  power_values_msg.battery_voltage = bat_v;
  power_values_msg.battery_current = (discharge_i * -1) + charge_i;
  power_values_msg.battery_power = bat_v * ((discharge_i * -1) + charge_i);
  power_values_msg.battery_charge_current = charge_i;
  power_values_msg.battery_charge_power = bat_v * charge_i;
  power_values_msg.battery_discharge_current = discharge_i;
  power_values_msg.battery_discharge_power = bat_v * discharge_i;
  power_values_msg.out19_voltage = out19_v;
  power_values_msg.out19_current = out19_i;
  power_values_msg.out19_power = out19_v * out19_i;
  pub_power_values.publish(&power_values_msg);
}

/*############################################################################
#               Precharge current when power module is running.     
#               Set value in mA for cut-off and precharge
#               allow 0.5-1.5A of 0-2A possible.
############################################################################*/
void set_precharge_current_running(const std_msgs::Int16 &msg)
{
  // check for limit values in miliamps
  if (msg.data < PRECHARGE_MA_MIN || msg.data > PRECHARGE_MA_MAX)
  {
    precharge_current_setpoint_running = PRECHARGE_TERM;
    nh.loginfo("Precharge/Cut-off current setpoint for running state is over limits. Default was set");
  }
  else
  {
    precharge_current_setpoint_running = int(float(msg.data + 300) * 0.05355); // 300mA correction(why?)  miliamps * milliamp value of 1/255
    nh.loginfo("Precharge/Cut-off current setpoint for running state was set");
  }

  EEPROM.put(precharge_current_setpoint_running_addr, precharge_current_setpoint_running);
}

/*############################################################################
#               Precharge current when power module is standby.     
#               Set value in mA for cut-off and precharge
#               allow 0.5-1.5A of 0-2A possible.
############################################################################*/
void set_precharge_current_standby(const std_msgs::Int16 &msg)
{
  // check for limit values in miliamps
  if (msg.data < PRECHARGE_MA_MIN || msg.data > PRECHARGE_MA_MAX)
  {
    precharge_current_setpoint_standby = PRECHARGE_TERM;
    nh.loginfo("Precharge/Cut off current setpoint for standby state is over limits. Default was set");
  }
  else
  {
    precharge_current_setpoint_standby = int(float(msg.data + 300) * 0.05355); // 300mA correction(why?)  miliamps * milliamp value of 1/255
    nh.loginfo("Precharge/Cut off current setpoint for standby state was set");
  }

  EEPROM.put(precharge_current_setpoint_standby_addr, precharge_current_setpoint_standby);
}

/*############################################################################
#               Charging current when power module is running.             
#               Value in miliamps for charging current 0-7A of max.
#               0-10A (10A=2.1V on arduino port).
############################################################################*/
void set_charge_current_running(const std_msgs::Int16 &msg)
{
  // check for limit values in miliamps
  if (msg.data < CHARGE_MA_MIN || msg.data > CHARGE_MA_MAX)
  {
    charge_current_setpoint_running = CHARGE_0A;
    nh.loginfo("Charge current setpoint for running state is over limits. Default was set");
  }
  else
  {
    charge_current_setpoint_running = int(float(msg.data) * 0.01063 * 1.23); //(0.01071) 1.23 je korekce, nevim proc
    nh.loginfo("Charge current setpoint for running state was set");
  }

  EEPROM.put(charge_current_setpoint_running_addr, charge_current_setpoint_running);
}

/*############################################################################
#               Charging current when power module is standby.             
#               Value in miliamps for charging current 0-7A of max.
#               0-10A (10A=2.1V on arduino port).
############################################################################*/
void set_charge_current_standby(const std_msgs::Int16 &msg)
{
  // check for limit values in miliamps
  if (msg.data < CHARGE_MA_MIN || msg.data > CHARGE_MA_MAX)
  {
    charge_current_setpoint_standby = CHARGE_0A;
    nh.loginfo("Charge current setpoint for stanby state is over limits. Default was set");
  }
  else
  {
    charge_current_setpoint_standby = int(float(msg.data) * 0.01063 * 1.23); //(0.01071) 1.23 je korekce, nevim proc
    nh.loginfo("Charge current setpoint for stanby state was set");
  }

  EEPROM.put(charge_current_setpoint_standby_addr, charge_current_setpoint_standby);
}

/*############################################################################
#                 Turn on/off charge switch
############################################################################*/
void set_charge_switch(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    battery_charge_switch = true;
    nh.loginfo("Battery charge switch is on.");
  }
  else
  {
    battery_charge_switch = false;
    nh.loginfo("Battery charge switch is off.");
  }
}

/*############################################################################
#                  Turn on/off discharge switch
############################################################################*/
void set_discharge_switch(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    battery_discharge_switch = true;
    nh.loginfo("Battery discharge switch is on.");
  }
  else
  {
    battery_discharge_switch = false;
    nh.loginfo("Battery discharge switch is off.");
  }
}

/*############################################################################
#                  Turn on/off motors output switch
############################################################################*/
void set_motor_switch(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    motors_output_switch = true;
    nh.loginfo("Motor output is on.");
  }
  else
  {
    motors_output_switch = false;
    nh.loginfo("Motor output is off.");
  }
}

/*############################################################################
#                 Turn on/off battery output switch
############################################################################*/
void set_bat_out_switch(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    battery_output_switch = true;
    nh.loginfo("Battery output is on.");
  }
  else
  {
    battery_output_switch = false;
    nh.loginfo("Battery output is off.");
  }
}

/*############################################################################
#                 Turn on/off 19V output switch
############################################################################*/
void set_19v_out_switch(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    output19_switch = true;
    dc19_enable = false; // 1=off , 0=on
    nh.loginfo("19V output is on.");
  }
  else
  {
    output19_switch = false;
    dc19_enable = true; // 1=off , 0=on
    nh.loginfo("19V output is off.");
  }
}

/*############################################################################
#                Set temperature setpoint for PID control
############################################################################*/
void set_temp_setpoint(const std_msgs::Float64 &msg)
{
  // check for limit values
  if (msg.data > 40.0)
  {
    temp_setpoint = 35.0;
    nh.loginfo("Temperature setpoint over limit set to 35 C deg.");
  }
  else
  {
    temp_setpoint = double(msg.data);
    nh.loginfo("Temperature setpoint was set.");
  }
  EEPROM.put(temp_setpoint_addr, temp_setpoint); // store value in eeprom
}

/*############################################################################
#                Set temperature2 setpoint for PID control
############################################################################*/
void set_temp2_setpoint(const std_msgs::Float64 &msg)
{
  // check for limit values
  if (msg.data > 45.0)
  {
    temp_ext_setpoint = 35.0;
    nh.loginfo("Temperature setpoint over limit set to 35 C deg.");
  }
  else
  {
    temp_ext_setpoint = double(msg.data);
    nh.loginfo("Temperature setpoint was set.");
  }
  EEPROM.put(temp_ext_setpoint_addr, temp_ext_setpoint); // store value in eeprom
}

/*############################################################################
#                 Set robot to sleep for time interval
############################################################################*/
void set_robot_sleep(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    sleep_status = true;
    sleep_charged_status = false;
    sleep_time_timer = millis();
    sleep_wait_before_standby_timer = millis();
    nh.loginfo("Sleep for time interval initiated.");
  }
  else
  {
    sleep_status = false;
    nh.loginfo("Sleep for time interval canceled.");
  }
}

/*############################################################################
#                 Set robot to sleep until charged
############################################################################*/
void set_sleep_until_charged(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    sleep_charged_status = true;
    sleep_status = false;
    sleep_wait_before_standby_timer = millis();
    nh.loginfo("Sleep until charged initiated.");
  }
  else
  {
    sleep_charged_status = false;
    nh.loginfo("Sleep until charged canceled.");
  }
}

/*############################################################################
#                 Set timeout in [ms] to shut down when on battery and do nothing
############################################################################*/
void set_standby_timeout_discharging(const std_msgs::UInt64 &msg)
{
  if (msg.data)
  {
    standby_timeout_discharging = msg.data;
    nh.loginfo("Timeout to shutdown on battery when standby, was set.");
    EEPROM.put(standby_timeout_discharging_addr, standby_timeout_discharging); // store value in eeprom
  }
}

/*############################################################################
#                 How long to sleep. Time in [ms].
############################################################################*/
void set_sleep_time(const std_msgs::UInt64 &msg)
{
  if (msg.data)
  {
    sleep_time_millis = msg.data;
    nh.loginfo("Sleep time was set.");
    EEPROM.put(sleep_time_millis_addr, sleep_time_millis); // store value in eeprom
  }
}

/*############################################################################
#                 Give the robot time to shutdown it self. 
#                 Wait before go to stanby. Time in [ms].
############################################################################*/
void set_sleep_wait_before_standby(const std_msgs::UInt64 &msg)
{
  if (msg.data)
  {
    sleep_wait_before_standby_millis = msg.data;
    nh.loginfo("Sleep time to wait before standby was set.");
    EEPROM.put(sleep_wait_before_standby_millis_addr, sleep_wait_before_standby_millis); // store value in eeprom
  }
}

/*############################################################################
#                 Set time in [ms] to wait after charged before turn on.
#                 Relax the battery after charging.
############################################################################*/
void set_sleep_wait_charged_offset(const std_msgs::UInt64 &msg)
{
  if (msg.data)
  {
    sleep_wait_charged_offset_millis = msg.data;
    nh.loginfo("Sleep time ofset to wake up after charged was set.");
    EEPROM.put(sleep_wait_charged_offset_millis_addr, sleep_wait_charged_offset_millis);// store value in eeprom
  }
}

