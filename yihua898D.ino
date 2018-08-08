/*
 * This is a custom firmware for my 'Youyue 858D+' hot-air soldering station.
 * It may or may not be useful to you, always double check if you use it.
 *
 * V2.46
 *
 * 2015/16 - Robert Spitzenpfeil
 * 2015    - Moritz Augsburger
 *
 * License: GNU GPL v2
 *
 *
 * Developed for / tested on by Robert Spitzenpfeil:
 * -------------------------------------------------
 *
 * Date:  2015-02-01
 * PCB version: 858D V6.0
 * Date code:   20140415
 *
 * Developed for / tested on by Moritz Augsburger:
 * -----------------------------------------------
 *
 * Date:  2015-02-01
 * PCB version: 858D V6.0
 * Date code:   20140415
 * 
 * Reported to work with (I did not test these myself):
 * ----------------------------------------------------
 *
 * PCB version: 858D V4.3
 * Date code:   20130529
 * HW mods:     not tested!
 *
 * ---
 *
 * PCB version: 858D V4.10
 * Date code: 20140112
 * HW mods: not tested!
 *
 */

/*
 *  Make sure to read and understand '/Docs/modes_of_operation.txt'
 * 
 * Change options in the .h file
 *
 */

#define FW_MAJOR_V 2
#define FW_MINOR_V_A 4
#define FW_MINOR_V_B 6
/*
 * #21: AREF <--- about 2.5V as analog reference for ADC
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>
#include "TM1628.h"
#include "yihua898D.h"

#ifdef USE_WATCHDOG
/* allocate memory for a signature string which will be stored in RAM after
 *  powerup  */
const char wdt_signature [] = "WDT_RESET";
char * p = (char *) malloc(sizeof(wdt_signature));
#endif

// define a module on data pin 8, clock pin 9 and strobe pin 7
TM1628 tm1628(8, 9, 7);

// HOT AIR configuration
HA_CFG ha_cfg = {
  /* p_gain */           { 0, 999, P_GAIN_DEFAULT, P_GAIN_DEFAULT, 2, 3 },  // min, max, default, value, eep_addr_high, eep_addr_low
  /* i_gain */           { 0, 999, I_GAIN_DEFAULT, I_GAIN_DEFAULT, 4, 5 },
  /* d_gain */           { 0, 999, D_GAIN_DEFAULT, D_GAIN_DEFAULT, 6, 7 },
  /* i_thresh */         { 0, 100, I_THRESH_DEFAULT, I_THRESH_DEFAULT, 8, 9 },
  /* temp_offset_corr */ { -100, 100, TEMP_OFFSET_CORR_DEFAULT, TEMP_OFFSET_CORR_DEFAULT, 10, 11 },
  /* temp_setpoint */    { 50, 500, TEMP_SETPOINT_DEFAULT, TEMP_SETPOINT_DEFAULT, 12, 13 },
  /* temp_averages */    { 100, 999, TEMP_AVERAGES_DEFAULT, TEMP_AVERAGES_DEFAULT, 14, 15 },
  /* slp_timeout */      { 0, 30, SLP_TIMEOUT_DEFAULT, SLP_TIMEOUT_DEFAULT, 16, 17 },
  /* fan_only */         { 0, 1, 0, 0, 26, 27 },
  /* display_adc_raw */  { 0, 1, 0, 0, 28, 29 },
#ifdef CURRENT_SENSE_MOD
  /* fan_current_min */  { 0, 999, FAN_CURRENT_MIN_DEFAULT, FAN_CURRENT_MIN_DEFAULT, 22, 23 },
  /* fan_current_max */  { 0, 999, FAN_CURRENT_MAX_DEFAULT, FAN_CURRENT_MAX_DEFAULT, 24, 25 },
#elif SPEED_SENSE_MOD
  //
  // See youyue858d.h if you want to use the 'FAN-speed mod' (HW changes required)
  // The following 2 CPARAM lines need changes in that case
  //
  /* fan_speed_min */    { 120, 180, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, 18, 19 },
  /* fan_speed_max */    { 300, 400, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, 20, 21 },
#endif
};

CNTRL_STATE ha_state = {
  /* temp_inst                 */   0,
  /* temp_accu                 */   0,
  /* temp_average              */   0,
  /* temp_average_previous     */   0,
  /* heater_ctr                */   0,
  /* heater_duty_cycle         */   0,
  /* error                     */   0,
  /* error_accu                */   0,
  /* velocity                  */   0,
  /* PID_drive                 */   0,
  /* temp_setpoint_saved       */   1,
  /* temp_setpoint_saved_time  */   0,
  /* heater_start_time         */   0,
  /* adc_raw                   */   0,
};

// SOLDERING IRON configuration
SI_CFG si_cfg = {
  /* p_gain */           { 0, 999, P_GAIN_DEFAULT, P_GAIN_DEFAULT, 30, 31 },  // min, max, default, value, eep_addr_high, eep_addr_low
  /* i_gain */           { 0, 999, I_GAIN_DEFAULT, I_GAIN_DEFAULT, 32, 33 },
  /* d_gain */           { 0, 999, D_GAIN_DEFAULT, D_GAIN_DEFAULT, 34, 35 },
  /* i_thresh */         { 0, 100, I_THRESH_DEFAULT, I_THRESH_DEFAULT, 36, 37 },
  /* temp_offset_corr */ { -100, 100, TEMP_OFFSET_CORR_DEFAULT, TEMP_OFFSET_CORR_DEFAULT, 38, 39 },
  /* temp_setpoint */    { 50, 500, TEMP_SETPOINT_DEFAULT, TEMP_SETPOINT_DEFAULT, 40, 41 },
  /* temp_averages */    { 100, 999, TEMP_AVERAGES_DEFAULT, TEMP_AVERAGES_DEFAULT, 42, 43 },
  /* slp_timeout */      { 0, 30, SLP_TIMEOUT_DEFAULT, SLP_TIMEOUT_DEFAULT, 44, 45 },
  /* display_adc_raw */  { 0, 1, 0, 0, 46, 47 },
};

CNTRL_STATE si_state = {
  /* temp_inst                 */   0,
  /* temp_accu                 */   0,
  /* temp_average              */   0,
  /* temp_average_previous     */   0,
  /* heater_ctr                */   0,
  /* heater_duty_cycle         */   0,
  /* error                     */   0,
  /* error_accu                */   0,
  /* velocity                  */   0,
  /* PID_drive                 */   0,
  /* temp_setpoint_saved       */   1,
  /* temp_setpoint_saved_time  */   0,
  /* heater_start_time         */   0,
  /* adc_raw                   */   0,
};

volatile uint8_t key_state = 0;  // debounced and inverted key state: bit = 1: key pressed
volatile uint8_t key_press = 0; // key press detect
volatile uint8_t key_rpt = 0; // key long press and repeat

volatile uint8_t display_blink;

void setup()
{
#ifdef USE_WATCHDOG
  watchdog_off();
#endif

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("\nRESET");
#endif
  
  setup_HW();
  
  load_cfg();
 
  tm1628.begin(ON, LED_POW); 

#ifdef USE_WATCHDOG
  if (watchdog_check()) 
  {
    // there was a watchdog reset - should never ever happen
    HA_HEATER_OFF;
    FAN_ON;
#ifdef DEBUG
    Serial.println("WDT reset!");
#endif
    while (1) {
      tm1628.showStr(DISP_1,"rSt");
      delay(1000);
      tm1628.clear(DISP_1);
      delay(1000);
#ifdef DEBUG
      break;
#endif
    }
  }
#endif

  show_firmware_version();

  fan_test();

#ifdef USE_WATCHDOG
  test_F_CPU_with_watchdog();
  watchdog_on();
#endif
}

void loop() {
#ifdef DEBUG
    int32_t start_time = micros();
#endif

    static int32_t button_input_time = 0;
    static int32_t button_scann_time = 0;

    ha_state.adc_raw = analogRead(HA_TEMP_PIN); // need raw value later, store it here and avoid 2nd ADC read

    ha_state.temp_inst = ha_state.adc_raw + ha_cfg.temp_offset_corr.value;  // approx. temp in °C

    if (ha_state.temp_inst < 0) {
      ha_state.temp_inst = 0;
    }
    // pid loop / heater handling
    if (ha_cfg.fan_only.value == 1 || REEDSW_CLOSED) {
      HA_HEATER_OFF;
      ha_state.heater_start_time = millis();
      tm1628.clearDot(DISP_2,0);
    } else if (REEDSW_OPEN && (ha_cfg.temp_setpoint.value >= ha_cfg.temp_setpoint.value_min)
         && (ha_state.temp_average < MAX_TEMP_ERR) && ((millis() - ha_state.heater_start_time) < ((uint32_t) (ha_cfg.slp_timeout.value) * 60 * 1000))) {

      FAN_ON;

      ha_state.error = ha_cfg.temp_setpoint.value - ha_state.temp_average;
      ha_state.velocity = ha_state.temp_average_previous - ha_state.temp_average;

      if (abs(ha_state.error) < ha_cfg.i_thresh.value) {
        // if close enough to target temperature use PID control
        ha_state.error_accu += ha_state.error;
      } else {
        // otherwise only use PD control (avoids issues with ha_state.error_accu growing too large
        ha_state.error_accu = 0;
      }

      ha_state.PID_drive =
          ha_state.error * (ha_cfg.p_gain.value / P_GAIN_SCALING) + ha_state.error_accu * (ha_cfg.i_gain.value / I_GAIN_SCALING) +
          ha_state.velocity * (ha_cfg.d_gain.value / D_GAIN_SCALING);

      ha_state.heater_duty_cycle = (int16_t) (ha_state.PID_drive);

      if (ha_state.heater_duty_cycle > HEATER_DUTY_CYCLE_MAX) {
        ha_state.heater_duty_cycle = HEATER_DUTY_CYCLE_MAX;
      }

      if (ha_state.heater_duty_cycle < 0) {
        ha_state.heater_duty_cycle = 0;
      }

      if (ha_state.heater_ctr < ha_state.heater_duty_cycle) {
        tm1628.setDot(DISP_2,0);
        HA_HEATER_ON;
      } else {
        HA_HEATER_OFF;
        tm1628.clearDot(DISP_2,0);
      }

      ha_state.heater_ctr++;
      if (ha_state.heater_ctr == PWM_CYCLES) {
        ha_state.heater_ctr = 0;
      }
    } else {
      HA_HEATER_OFF;
      tm1628.clearDot(DISP_2,0);
    }

    static uint16_t temp_avg_ctr = 0;

    ha_state.temp_accu += ha_state.temp_inst;
    temp_avg_ctr++;

    if (temp_avg_ctr == (uint16_t) (ha_cfg.temp_averages.value)) {
      ha_state.temp_average_previous = ha_state.temp_average;
      ha_state.temp_average = ha_state.temp_accu / ha_cfg.temp_averages.value;
      ha_state.temp_accu = 0;
      temp_avg_ctr = 0;
    }
    // fan/cradle handling
    if (ha_state.temp_average >= FAN_ON_TEMP) {
      FAN_ON;
    } else if (REEDSW_CLOSED && ha_cfg.fan_only.value == 1 && (ha_state.temp_average <= FAN_OFF_TEMP_FANONLY)) {
      FAN_OFF;
    } else if (REEDSW_CLOSED && ha_cfg.fan_only.value == 0 && (ha_state.temp_average <= FAN_OFF_TEMP)) {
      FAN_OFF;
    } else if (REEDSW_OPEN) {
      FAN_ON;
    }
    // menu key handling
    if (millis() - button_scann_time > BUTTON_SCANN_CYCLE)
    {
      key_scann();
      button_scann_time = millis();
    }
    if (get_key_short(KEY_UP)) {
      button_input_time = millis();
      if (ha_cfg.temp_setpoint.value < ha_cfg.temp_setpoint.value_max) {
        ha_cfg.temp_setpoint.value++;
      }
      ha_state.temp_setpoint_saved = 0;
    } else if (get_key_short(KEY_DOWN)) {
      button_input_time = millis();
      if (ha_cfg.temp_setpoint.value > ha_cfg.temp_setpoint.value_min) {
        ha_cfg.temp_setpoint.value--;
      }
      ha_state.temp_setpoint_saved = 0;
    } else if (get_key_long_r(KEY_UP) || get_key_rpt_l(KEY_UP)) {
      button_input_time = millis();
      if (ha_cfg.temp_setpoint.value < (ha_cfg.temp_setpoint.value_max - 10)) {
        ha_cfg.temp_setpoint.value += 10;
      } else {
        ha_cfg.temp_setpoint.value = ha_cfg.temp_setpoint.value_max;
      }
      ha_state.temp_setpoint_saved = 0;

    } else if (get_key_long_r(KEY_DOWN) || get_key_rpt_l(KEY_DOWN)) {
      button_input_time = millis();

      if (ha_cfg.temp_setpoint.value > (ha_cfg.temp_setpoint.value_min + 10)) {
        ha_cfg.temp_setpoint.value -= 10;
      } else {
        ha_cfg.temp_setpoint.value = ha_cfg.temp_setpoint.value_min;
      }

      ha_state.temp_setpoint_saved = 0;
    } else if (get_key_common_l(KEY_UP | KEY_DOWN)) {
      HA_HEATER_OFF;  // security reasons, delay below!
#ifdef USE_WATCHDOG
      watchdog_off();
#endif
      delay(uint16_t(20.48 * (REPEAT_START - 3) + 1));
      if (get_key_long_r(KEY_UP | KEY_DOWN)) {
        change_config_parameter(&ha_cfg.p_gain, "P", DISP_2);
        change_config_parameter(&ha_cfg.i_gain, "I", DISP_2);
        change_config_parameter(&ha_cfg.d_gain, "d", DISP_2);
        change_config_parameter(&ha_cfg.i_thresh, "ItH", DISP_2);
        change_config_parameter(&ha_cfg.temp_offset_corr, "toF", DISP_2);
        change_config_parameter(&ha_cfg.temp_averages, "Avg", DISP_2);
        change_config_parameter(&ha_cfg.slp_timeout, "SLP", DISP_2);
        change_config_parameter(&ha_cfg.display_adc_raw, "Adc", DISP_2);
#ifdef CURRENT_SENSE_MOD
        change_config_parameter(&ha_cfg.fan_current_min, "FcL", DISP_2);
        change_config_parameter(&ha_cfg.fan_current_max, "FcH", DISP_2);
#elif SPEED_SENSE_MOD
        change_config_parameter(&ha_cfg.fan_speed_min, "FSL", DISP_2);
        change_config_parameter(&ha_cfg.fan_speed_max, "FSH", DISP_2);
#endif
      } else {
        get_key_press(KEY_UP | KEY_DOWN); // clear inp state
        ha_cfg.fan_only.value ^= 0x01;
        ha_state.temp_setpoint_saved = 0;
        if (ha_cfg.fan_only.value == 0) {
          button_input_time = millis(); // show set temp after disabling fan only mode
        }
        display_blink = 0;  // make sure we start displaying "FAn" or set temp
      }
#ifdef USE_WATCHDOG
      watchdog_on();
#endif
    }
    // security first!
    if (ha_state.temp_average >= MAX_TEMP_ERR) {
      // something might have gone terribly wrong
      HA_HEATER_OFF;
      FAN_ON;
#ifdef USE_WATCHDOG
      watchdog_off();
#endif
#ifdef DEBUG
      Serial.println("Hot air temperature meas. error!");
#endif
      while (1) {
        // stay here until the power is cycled
        // make sure the user notices the error by blinking "FAn"
        // and don't resume operation if the error goes away on its own
        //
        // possible reasons to be here:
        //
        // * wand is not connected (false temperature reading)
        // * thermo couple has failed
        // * true over-temperature condition
        //
        tm1628.showStr(DISP_2,"*C");
        delay(1000);
        tm1628.showStr(DISP_2,"Err");
        delay(2000);
        tm1628.clear(DISP_2);
        delay(1000);
      }
    }
    // display output
    if ((millis() - button_input_time) < SHOW_SETPOINT_TIMEOUT) {
      if (display_blink < 5) {
        tm1628.clear(DISP_2);
      } else {
        tm1628.showNum(DISP_2,ha_cfg.temp_setpoint.value);  // show temperature setpoint
      }
    } else {
      if (ha_state.temp_setpoint_saved == 0) {
        set_eeprom_saved_dot(DISP_2);
        eep_save(&ha_cfg.temp_setpoint);
        eep_save(&ha_cfg.fan_only);
        ha_state.temp_setpoint_saved_time = millis();
        ha_state.temp_setpoint_saved = 1;
      } else if (ha_state.temp_average <= SAFE_TO_TOUCH_TEMP) {
        if (ha_cfg.fan_only.value == 1) {
          tm1628.showStr(DISP_2,"FAn");
        } else {
          tm1628.showStr(DISP_2,"---");
        }
      } else if (ha_cfg.fan_only.value == 1) {
        if (display_blink < 20) {
          tm1628.showStr(DISP_2,"FAn");
        } else {
          tm1628.showNum(DISP_2,ha_state.temp_average);
        }
      } else if (ha_cfg.display_adc_raw.value == 1) {
        tm1628.showNum(DISP_2,ha_state.adc_raw);
      } else if (abs((int16_t) (ha_state.temp_average) - (int16_t) (ha_cfg.temp_setpoint.value)) < TEMP_REACHED_MARGIN) {
        tm1628.showNum(DISP_2,ha_cfg.temp_setpoint.value);  // avoid showing insignificant fluctuations on the display (annoying)
      } else {
        tm1628.showNum(DISP_2,ha_state.temp_average);
      }
    }

    if ((millis() - ha_state.temp_setpoint_saved_time) > 500) {
      clear_eeprom_saved_dot(DISP_2);
    }

    tm1628.update();

#if defined(WATCHDOG_TEST) && defined(USE_WATCHDOG)
    // watchdog test
    if (ha_state.temp_average > 100) {
      delay(150);
    }
#endif

#ifdef USE_WATCHDOG
    wdt_reset();
#endif

#ifdef DEBUG
    int32_t stop_time = micros();
    Serial.print("Loop time: ");
    Serial.println(stop_time - start_time);
#endif
}

void setup_HW(void)
{
  HA_HEATER_INIT;
  HA_HEATER_OFF;
  
  SI_HEATER_INIT;
  SI_HEATER_OFF;

  FAN_INIT;
  FAN_OFF;
  
  REEDSW_INIT;
  HA_SW_INIT;
  SI_SW_INIT;
  
  pinMode(HA_TEMP_PIN, INPUT);
  pinMode(SI_TEMP_PIN, INPUT);

#ifdef CURRENT_SENSE_MOD
  pinMode(FAN_CURRENT_PIN, INPUT);  // set as input
#elif SPEED_SENSE_MOD
  pinMode(FAN_SPEED_PIN, INPUT);  // set as input
#endif

  analogReference(EXTERNAL);  // use external 2.5V as ADC reference voltage (VCC / 2)

  if (EEPROM.read(0) != 0x22) {
    // check if the firmware was just flashed and the EEPROM is therefore empty
    // assumption: full chip erase with ISP programmer (preserve eeprom fuse NOT set!)
    // if so, restore default parameter values & write a 'hint' to address 0
    restore_default_conf();
    EEPROM.write(0, 0x22);
#ifdef DEBUG
  Serial.println("Default config loaded");
#endif
  }
  
  key_scann();

  if (get_key_state(KEY_DOWN) && get_key_state(KEY_UP)) {
    restore_default_conf();
  } else if (get_key_state(KEY_UP)) {
    tm1628.showStr(DISP_2,"FAn");
    delay(1000);
    tm1628.showStr(DISP_2,"tSt");
    delay(1000);
    FAN_ON;
    while (1) {
      uint16_t fan;
      delay(500);
#ifdef CURRENT_SENSE_MOD
      fan = analogRead(FAN_CURRENT_PIN);
      tm1628.showNum(DISP_2,fan);
#elif SPEED_SENSE_MOD
      fan = analogRead(FAN_SPEED_PIN);
      tm1628.showNum(DISP_2,fan);
#endif        //CURRENT_SENSE_MOD
      
    }
  }
}

void load_cfg(void)
{
  eep_load(&ha_cfg.p_gain);
  eep_load(&ha_cfg.i_gain);
  eep_load(&ha_cfg.d_gain);
  eep_load(&ha_cfg.i_thresh);
  eep_load(&ha_cfg.temp_offset_corr);
  eep_load(&ha_cfg.temp_setpoint);
  eep_load(&ha_cfg.temp_averages);
  eep_load(&ha_cfg.slp_timeout);
  eep_load(&ha_cfg.fan_only);
  eep_load(&ha_cfg.display_adc_raw);
#ifdef CURRENT_SENSE_MOD
  eep_load(&ha_cfg.fan_current_min);
  eep_load(&ha_cfg.fan_current_max);
#elif SPEED_SENSE_MOD
  eep_load(&ha_cfg.fan_speed_min);
  eep_load(&ha_cfg.fan_speed_max);
#endif

  eep_load(&si_cfg.p_gain);
  eep_load(&si_cfg.i_gain);
  eep_load(&si_cfg.d_gain);
  eep_load(&si_cfg.i_thresh);
  eep_load(&si_cfg.temp_offset_corr);
  eep_load(&si_cfg.temp_setpoint);
  eep_load(&si_cfg.temp_averages);
  eep_load(&si_cfg.slp_timeout);
  eep_load(&si_cfg.display_adc_raw);
}

void change_config_parameter(CPARAM * param, const char *string, uint8_t disp)
{
  tm1628.showStr(disp,string);
  delay(750);   // let the user read what is shown

  uint8_t loop = 1;

  while (loop == 1) {
    if (get_key_short(KEY_UP)) {
      if (param->value < param->value_max) {
        param->value++;
      }
    } else if (get_key_short(KEY_DOWN)) {
      if (param->value > param->value_min) {
        param->value--;
      }
    } else if (get_key_long_r(KEY_UP) || get_key_rpt_l(KEY_UP)) {
      if (param->value < param->value_max - 10) {
        param->value += 10;
      }
    } else if (get_key_long_r(KEY_DOWN) || get_key_rpt_l(KEY_DOWN)) {
      if (param->value > param->value_min + 10) {
        param->value -= 10;
      }
    } else if (get_key_common(KEY_UP | KEY_DOWN)) {
      loop = 0;
    }

    tm1628.showNum(disp,param->value);
  }
  set_eeprom_saved_dot(disp);
  eep_save(param);
  delay(1000);
  clear_eeprom_saved_dot(disp);
}

void eep_save(CPARAM * param)
{
  // make sure NOT to save invalid parameter values
  if ((param->value >= param->value_min) && (param->value <= param->value_max)) {
    // nothing to do
  } else {
    // reset to sensible minimum
    param->value = param->value_default;
  }
  EEPROM.update(param->eep_addr_high, highByte(param->value));
  EEPROM.update(param->eep_addr_low, lowByte(param->value));
}

void eep_load(CPARAM * param)
{
  int16_t tmp = (EEPROM.read(param->eep_addr_high) << 8) | EEPROM.read(param->eep_addr_low);

  // make sure NOT to restore invalid parameter values
  if ((tmp >= param->value_min) && (tmp <= param->value_max)) {
    // the value was good, so we use it
    param->value = tmp;
  } else {
    // reset to sensible value
    param->value = param->value_default;
  }
}

void restore_default_conf(void)
{
  ha_cfg.p_gain.value = ha_cfg.p_gain.value_default;
  ha_cfg.i_gain.value = ha_cfg.i_gain.value_default;
  ha_cfg.d_gain.value = ha_cfg.d_gain.value_default;
  ha_cfg.i_thresh.value = ha_cfg.i_thresh.value_default;
  ha_cfg.temp_offset_corr.value = ha_cfg.temp_offset_corr.value_default;
  ha_cfg.temp_setpoint.value = ha_cfg.temp_setpoint.value_default;
  ha_cfg.temp_averages.value = ha_cfg.temp_averages.value_default;
  ha_cfg.slp_timeout.value = ha_cfg.slp_timeout.value_default;
  ha_cfg.fan_only.value = ha_cfg.fan_only.value_default;
  ha_cfg.display_adc_raw.value = ha_cfg.display_adc_raw.value_default;
#ifdef CURRENT_SENSE_MOD
  ha_cfg.fan_current_min.value = ha_cfg.fan_current_min.value_default;
  ha_cfg.fan_current_max.value = ha_cfg.fan_current_max.value_default;
#elif SPEED_SENSE_MOD
  ha_cfg.fan_speed_min.value = ha_cfg.fan_speed_min.value_default;
  ha_cfg.fan_speed_max.value = ha_cfg.fan_speed_max.value_default;
#endif

  eep_save(&ha_cfg.p_gain);
  eep_save(&ha_cfg.i_gain);
  eep_save(&ha_cfg.d_gain);
  eep_save(&ha_cfg.i_thresh);
  eep_save(&ha_cfg.temp_offset_corr);
  eep_save(&ha_cfg.temp_setpoint);
  eep_save(&ha_cfg.temp_averages);
  eep_save(&ha_cfg.slp_timeout);
  eep_save(&ha_cfg.fan_only);
  eep_save(&ha_cfg.display_adc_raw);
#ifdef CURRENT_SENSE_MOD
  eep_save(&ha_cfg.fan_current_min);
  eep_save(&ha_cfg.fan_current_max);
#elif SPEED_SENSE_MOD
  eep_save(&ha_cfg.fan_speed_min);
  eep_save(&ha_cfg.fan_speed_max);
#endif


  si_cfg.p_gain.value = si_cfg.p_gain.value_default;
  si_cfg.i_gain.value = si_cfg.i_gain.value_default;
  si_cfg.d_gain.value = si_cfg.d_gain.value_default;
  si_cfg.i_thresh.value = si_cfg.i_thresh.value_default;
  si_cfg.temp_offset_corr.value = si_cfg.temp_offset_corr.value_default;
  si_cfg.temp_setpoint.value = si_cfg.temp_setpoint.value_default;
  si_cfg.temp_averages.value = si_cfg.temp_averages.value_default;
  si_cfg.slp_timeout.value = si_cfg.slp_timeout.value_default;
  si_cfg.display_adc_raw.value = si_cfg.display_adc_raw.value_default;

  eep_save(&si_cfg.p_gain);
  eep_save(&si_cfg.i_gain);
  eep_save(&si_cfg.d_gain);
  eep_save(&si_cfg.i_thresh);
  eep_save(&si_cfg.temp_offset_corr);
  eep_save(&si_cfg.temp_setpoint);
  eep_save(&si_cfg.temp_averages);
  eep_save(&si_cfg.slp_timeout);
  eep_save(&si_cfg.display_adc_raw);
}

void set_eeprom_saved_dot(uint8_t disp)
{
  tm1628.setDot(disp,1);  // dig1.dot
  tm1628.update();
}

void clear_eeprom_saved_dot(uint8_t disp)
{
  tm1628.clearDot(disp,1);  // dig1.dot
  tm1628.update();
}

void fan_test(void)
{
  HA_HEATER_OFF;

  // if the wand is not in the cradle when powered up, go into a safe mode
  // and display an error
  while (!REEDSW_CLOSED) {
    tm1628.showStr(DISP_2,"crA");
    delay(1000);
    tm1628.showStr(DISP_2,"dLE");
    delay(2000);
    tm1628.clear(DISP_2);
    delay(1000);
#ifdef DEBUG
    Serial.println("Cradle error!");
    break;
#endif
  }

#ifdef CURRENT_SENSE_MOD
  uint16_t fan_current;
  FAN_ON;
  delay(3000);
  fan_current = analogRead(FAN_CURRENT_PIN);

  if ((fan_current < (uint16_t) (ha_cfg.fan_current_min.value)) || (fan_current > (uint16_t) (ha_cfg.fan_current_max.value))) {
    // the fan is not working as it should
    FAN_OFF;
    while (1) {
      tm1628.showStr(DISP_2,"FAn");
      delay(1000);
      tm1628.showStr(DISP_2,"cur");
      delay(2000);
      tm1628.clear(DISP_2);
      delay(1000);
#ifdef DEBUG
      Serial.println("Fan current meas. error!");
      break;      
#endif
    }
  }
#elif SPEED_SENSE_MOD       
  uint16_t fan_speed;
  FAN_ON;
  delay(3000);
  fan_speed = analogRead(FAN_SPEED_PIN);

  if ((fan_speed < (uint16_t) (ha_cfg.fan_speed_min.value)) || (fan_speed > (uint16_t) (ha_cfg.fan_speed_max.value))) {
    // the fan is not working as it should
    FAN_OFF;
    while (1) {
      tm1628.showStr(DISP_2,"FAn");
      delay(1000);
      tm1628.showStr(DISP_2,"SPd");
      delay(2000);
      tm1628.clear(DISP_2);
      delay(1000);
#ifdef DEBUG
      Serial.println("Fan speed meas. error!");
      break;
#endif
    }
  }
#endif

  FAN_OFF;

}

void show_firmware_version(void)
{
  tm1628.setDig(DISP_1,0,FW_MINOR_V_B); // dig0
  tm1628.setDig(DISP_1,1,FW_MINOR_V_A); // dig1
  tm1628.setDig(DISP_1,2,FW_MAJOR_V); // dig2
  tm1628.setDot(DISP_1,1);  // dig1.dot
  tm1628.setDot(DISP_1,2);  // dig2.dot
  tm1628.update();
#ifdef DEBUG
  tm1628.showStr(DISP_2,"dbg");
  Serial.print("FW v");
  Serial.print(FW_MAJOR_V);
  Serial.print(".");
  Serial.print(FW_MINOR_V_A);
  Serial.print(".");
  Serial.print(FW_MINOR_V_B);
  Serial.println();
#endif
  delay(2000);
  tm1628.clear(DISP_ALL);
}

void key_scann(void)
{  
  static uint8_t rpt;
  uint8_t new_key_state;

  new_key_state = tm1628.getButtons();
  key_press |= key_state ^ new_key_state;
  key_state = new_key_state; 
  //TODO  
  //key_press &= key_state; // Remove old key press events
    
  if ((key_state & REPEAT_MASK) == 0) // check repeat function
    rpt = REPEAT_START; // start delay
  if (--rpt == 0) {
    rpt = REPEAT_NEXT;  // repeat delay
    key_rpt |= key_state & REPEAT_MASK;
  }
  
  if (++display_blink > 50)
    display_blink = 0;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint8_t get_key_press(uint8_t key_mask)
{
  cli();      // read and clear atomic !
  key_mask &= key_press;  // read key(s)
  key_press ^= key_mask;  // clear key(s)
  sei();
  return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint8_t get_key_rpt(uint8_t key_mask)
{
  cli();      // read and clear atomic !
  key_mask &= key_rpt;  // read key(s)
  key_rpt ^= key_mask;  // clear key(s)
  sei();
  return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state(uint8_t key_mask)
{
  key_mask &= key_state;
  return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_short(uint8_t key_mask)
{
  cli();      // read key state and key press atomic !
  return get_key_press(~key_state & key_mask);
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_long(uint8_t key_mask)
{
  return get_key_press(get_key_rpt(key_mask));
}

uint8_t get_key_long_r(uint8_t key_mask)
{       // if repeat function needed
  return get_key_press(get_key_rpt(key_press & key_mask));
}

uint8_t get_key_rpt_l(uint8_t key_mask)
{       // if long function needed
  return get_key_rpt(~key_press & key_mask);
}

uint8_t get_key_common(uint8_t key_mask)
{
  return get_key_press((key_press & key_mask) == key_mask ? key_mask : 0);
}

uint8_t get_key_common_l(uint8_t key_mask)
{
  return get_key_state((key_press & key_mask) == key_mask ? key_mask : 0);
}

#ifdef USE_WATCHDOG
void watchdog_off(void)
{
  wdt_reset();
  wdt_disable();
}

void watchdog_on(void)
{
  wdt_reset();
  wdt_enable(WDTO_120MS);
}

uint8_t watchdog_check(void) 
{
  if (strcmp (p, wdt_signature) == 0) 
  { // signature is in RAM this was reset
    return 1;
  }
  else 
  {  // signature not in RAM this was a power on
    // add the signature to be retained in memory during reset
    memcpy(p, wdt_signature, sizeof(wdt_signature));  // copy signature into RAM
    return 0;
  }
}

void test_F_CPU_with_watchdog(void)
{
/*
 * Hopefully cause a watchdog reset if the CKDIV8 FUSE is set (F_CPU 1MHz instead of 8MHz)
 *
 */
  wdt_reset();
  wdt_enable(WDTO_120MS);
  delay(40);    // IF "CKDIV8" fuse is erroneously set, this should delay by 8x40 = 320ms & cause the dog to bite!

  watchdog_off();   // IF we got to here, F_CPU is OK.
}
#endif
