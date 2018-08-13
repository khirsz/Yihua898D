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
 * #21: AREF <--- about 3.3V as analogue reference for ADC, i.e. from Arduino 3.3V pin.
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
DEV_CFG ha_cfg = {
  /* device type */      DEV_HA,
  /* display number */   DISP_2,
  /* p_gain */           { 0, 999, P_GAIN_DEFAULT, P_GAIN_DEFAULT, 2, 3, "P"},  // min, max, default, value, eep_addr_high, eep_addr_low, name
  /* i_gain */           { 0, 999, I_GAIN_DEFAULT, I_GAIN_DEFAULT, 4, 5, "I"},
  /* d_gain */           { 0, 999, D_GAIN_DEFAULT, D_GAIN_DEFAULT, 6, 7, "d"},
  /* i_thresh */         { 0, 100, I_THRESH_DEFAULT, I_THRESH_DEFAULT, 8, 9, "ItH"},
  /* temp_offset_corr */ { -100, 100, TEMP_OFFSET_CORR_DEFAULT, TEMP_OFFSET_CORR_DEFAULT, 10, 11, "toF"},
  /* temp_averages */    { 100, 999, TEMP_AVERAGES_DEFAULT, TEMP_AVERAGES_DEFAULT, 14, 15, "Avg"},
  /* slp_timeout */      { 0, 30, SLP_TIMEOUT_DEFAULT, SLP_TIMEOUT_DEFAULT, 16, 17, "SLP"},
  /* display_adc_raw */  { 0, 1, 0, 0, 28, 29, "Adc"},
#ifdef CURRENT_SENSE_MOD
  /* fan_current_min */  { 0, 999, FAN_CURRENT_MIN_DEFAULT, FAN_CURRENT_MIN_DEFAULT, 22, 23, "FcL"},
  /* fan_current_max */  { 0, 999, FAN_CURRENT_MAX_DEFAULT, FAN_CURRENT_MAX_DEFAULT, 24, 25, "FcH"},
#elif defined(SPEED_SENSE_MOD)
  //
  // See youyue858d.h if you want to use the 'FAN-speed mod' (HW changes required)
  // The following 2 CPARAM lines need changes in that case
  //
  /* fan_speed_min */    { 120, 180, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, 18, 19, "FSL"},
  /* fan_speed_max */    { 300, 400, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, 20, 21, "FSH"},
#endif
  // Not configurable in setting change mode
  /* temp_setpoint */    { 50, 500, TEMP_SETPOINT_DEFAULT, TEMP_SETPOINT_DEFAULT, 12, 13, "SP"},
  /* fan_only */         { 0, 1, 0, 0, 26, 27, "FnO"},
};
CPARAM * ha_set_order[] = {&ha_cfg.p_gain, &ha_cfg.i_gain, &ha_cfg.d_gain, &ha_cfg.i_thresh,
                                      &ha_cfg.temp_offset_corr, &ha_cfg.temp_averages, &ha_cfg.slp_timeout, &ha_cfg.display_adc_raw,
#ifdef CURRENT_SENSE_MOD
                                      &ha_cfg.fan_current_min, &ha_cfg.fan_current_max,
#elif defined(SPEED_SENSE_MOD)
                                      &ha_cfg.fan_speed_min, &ha_cfg.fan_speed_max,
#endif
                                      };
CNTRL_STATE ha_state;

// SOLDERING IRON configuration
DEV_CFG si_cfg = {
  /* device type */      DEV_SI,
  /* display number */   DISP_1,
  /* p_gain */           { 0, 999, P_GAIN_DEFAULT, P_GAIN_DEFAULT, 30, 31, "P"},  // min, max, default, value, eep_addr_high, eep_addr_low, name
  /* i_gain */           { 0, 999, I_GAIN_DEFAULT, I_GAIN_DEFAULT, 32, 33, "I"},
  /* d_gain */           { 0, 999, D_GAIN_DEFAULT, D_GAIN_DEFAULT, 34, 35, "d"},
  /* i_thresh */         { 0, 100, I_THRESH_DEFAULT, I_THRESH_DEFAULT, 36, 37, "ItH"},
  /* temp_offset_corr */ { -100, 100, TEMP_OFFSET_CORR_DEFAULT, TEMP_OFFSET_CORR_DEFAULT, 38, 39, "toF"},
  /* temp_averages */    { 100, 999, TEMP_AVERAGES_DEFAULT, TEMP_AVERAGES_DEFAULT, 40, 41, "Avg"},
  /* slp_timeout */      CPARAM_NULL,
  /* display_adc_raw */  { 0, 1, 0, 0, 44, 45, "Adc"},
#ifdef CURRENT_SENSE_MOD
  /* fan_current_min */  CPARAM_NULL,
  /* fan_current_max */  CPARAM_NULL,
#elif defined(SPEED_SENSE_MOD)
  //
  // See youyue858d.h if you want to use the 'FAN-speed mod' (HW changes required)
  // The following 2 CPARAM lines need changes in that case
  //
  /* fan_speed_min */    CPARAM_NULL,
  /* fan_speed_max */    CPARAM_NULL,
#endif
  // Not configurable in setting change mode
  /* temp_setpoint */    { 50, 500, TEMP_SETPOINT_DEFAULT, TEMP_SETPOINT_DEFAULT, 46, 47, "SP"},
  /* fan_only */         CPARAM_NULL,
};
CPARAM * si_set_order[] = {&si_cfg.p_gain, &si_cfg.i_gain, &si_cfg.d_gain, &si_cfg.i_thresh,
                                      &si_cfg.temp_offset_corr, &si_cfg.temp_averages, &si_cfg.display_adc_raw,};
CNTRL_STATE si_state;

volatile uint8_t key_state = 0;  // debounced and inverted key state: bit = 1: key pressed
volatile uint8_t key_state_l = 0; // key long press
volatile uint8_t key_state_s = 0; // key short press
volatile uint8_t sw_state = 0; // debounced switch state: bit = 1: sw on

void setup()
{
#ifdef USE_WATCHDOG
  watchdog_off();
#endif

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("\nRESET");
#endif
  
  tm1628.begin(ON, LED_POW); 
    
  setup_HW();

  init_state(&ha_state);
  init_state(&si_state);
  
  load_cfg();

#ifdef USE_WATCHDOG
  if (watchdog_check()) 
  {
    // there was a watchdog reset - should never ever happen
    HEATERS_OFF;
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

#ifdef USE_WATCHDOG
  test_F_CPU_with_watchdog();
  watchdog_on();
#endif

  key_event_clear();
}

void loop() {
    static uint32_t button_scan_time = 0;

#ifdef USE_WATCHDOG
    wdt_reset();
#endif

    if (millis() - button_scan_time > BUTTON_SCANN_CYCLE)
    {
      key_scan();
      button_scan_time = millis();
    }

    tm1628.update();  

#if defined(WATCHDOG_TEST) && defined(USE_WATCHDOG)
    // watchdog test
    if (ha_state.temp_average > 100) {
      delay(150);
    }
#endif

    //Control HA
    dev_cntrl(&ha_cfg, &ha_state);
    //Control SI
    dev_cntrl(&si_cfg, &si_state);
    //UI
    UI_hndl();
}

void dev_cntrl(DEV_CFG *pDev_cfg, CNTRL_STATE *pDev_state)
{
  uint8_t sw_mask;
  uint8_t analog_pin;
#ifdef DEBUG
  int32_t start_time = micros();
#endif
  if (pDev_cfg->dev_type == DEV_HA) {
    // HA device
    sw_mask = HA_SW;
    analog_pin = HA_TEMP_PIN;
  } else {
    // SI device
    sw_mask = SI_SW;
    analog_pin = SI_TEMP_PIN;
  }
      
  if (!pDev_state->enabled && get_sw_state(sw_mask))
  {
    // Enable device
    pDev_state->enabled = 1;
#ifdef DEBUG
    if (pDev_cfg->dev_type == DEV_HA) {
      Serial.println("HA on!");
    } else {
      Serial.println("SI on!");
    }
#endif
    if (pDev_cfg->dev_type == DEV_HA) { // Only for HA
      pDev_state->test_state = HA_test(TEST_INIT);
    }
       
  } else if (pDev_state->enabled && !get_sw_state(sw_mask)) 
  {
    // Disable Device
    pDev_state->enabled = 0;
    if (pDev_cfg->dev_type == DEV_HA) {
      HA_HEATER_OFF;
      FAN_OFF;
    } else {
      SI_HEATER_OFF;
    }
#ifdef DEBUG
    if (pDev_cfg->dev_type == DEV_HA) {
      Serial.println("HA off!");
    } else {
      Serial.println("SI off!");
    }
#endif
    tm1628.clear(pDev_cfg->disp_n);
    // Clear state
    init_state(&ha_state);
  }

  if (pDev_state->enabled)
  {
    if (pDev_state->test_state != TEST_ALL_OK) { //Startup test is still running
      pDev_state->test_state = HA_test(pDev_state->test_state);
      return;
    }
    pDev_state->adc_raw = analogRead(analog_pin); // need raw value later, store it here and avoid 2nd ADC read
#ifdef DEBUG
    if (start_time % 1000 == 0)
    {
      if (pDev_cfg->dev_type == DEV_HA) {
        Serial.print("HA adc=");
      } else {
        Serial.print("SI adc=");
      }
      Serial.print(pDev_state->adc_raw);
      Serial.println();
    }
#endif

    pDev_state->temp_inst = pDev_state->adc_raw + pDev_cfg->temp_offset_corr.value;  // approx. temp in °C

    if (pDev_state->temp_inst < 0) {
      pDev_state->temp_inst = 0;
    }
    // pid loop / heater handling
    if (pDev_cfg->dev_type == DEV_HA && (pDev_cfg->fan_only.value == 1 || REEDSW_CLOSED)) { // Only for HA
      HA_HEATER_OFF;
      //TODO
      pDev_state->heater_start_time = millis();
      tm1628.clearDot(pDev_cfg->disp_n,0);
    } else if ((pDev_cfg->dev_type == DEV_SI || REEDSW_OPEN) 
         && (pDev_cfg->temp_setpoint.value >= pDev_cfg->temp_setpoint.value_min)
         && (pDev_state->temp_average < MAX_TEMP_ERR) 
         && (pDev_cfg->dev_type == DEV_SI || ((millis() - pDev_state->heater_start_time) < ((uint32_t) (pDev_cfg->slp_timeout.value) * 60 * 1000)))) {
      // Run PID regulation
      if (pDev_cfg->dev_type == DEV_HA) {
        FAN_ON;
      }

      pDev_state->error = pDev_cfg->temp_setpoint.value - pDev_state->temp_average;
      pDev_state->velocity = pDev_state->temp_average_previous - pDev_state->temp_average;

      if (abs(pDev_state->error) < pDev_cfg->i_thresh.value) {
        // if close enough to target temperature use PID control
        pDev_state->error_accu += pDev_state->error;
      } else {
        // otherwise only use PD control (avoids issues with pDev_state->error_accu growing too large
        pDev_state->error_accu = 0;
      }

      pDev_state->PID_drive =
          pDev_state->error * (pDev_cfg->p_gain.value / P_GAIN_SCALING) + pDev_state->error_accu * (pDev_cfg->i_gain.value / I_GAIN_SCALING) +
          pDev_state->velocity * (pDev_cfg->d_gain.value / D_GAIN_SCALING);

      pDev_state->heater_duty_cycle = (int16_t) (pDev_state->PID_drive);

      if (pDev_state->heater_duty_cycle > HEATER_DUTY_CYCLE_MAX) {
        pDev_state->heater_duty_cycle = HEATER_DUTY_CYCLE_MAX;
      }

      if (pDev_state->heater_duty_cycle < 0) {
        pDev_state->heater_duty_cycle = 0;
      }

      if (pDev_state->heater_ctr < pDev_state->heater_duty_cycle) {
        tm1628.setDot(pDev_cfg->disp_n,0);
        if (pDev_cfg->dev_type == DEV_HA) {
          HA_HEATER_ON;
        } else {
          SI_HEATER_ON;
        }
      } else {
        if (pDev_cfg->dev_type == DEV_HA) {
          HA_HEATER_OFF;
        } else {
          SI_HEATER_OFF;
        }
        tm1628.clearDot(pDev_cfg->disp_n,0);
      }

      pDev_state->heater_ctr++;
      if (pDev_state->heater_ctr == PWM_CYCLES) {
        pDev_state->heater_ctr = 0;
      }
    } else {
      if (pDev_cfg->dev_type == DEV_HA) {
        HA_HEATER_OFF;
      } else {
        SI_HEATER_OFF;
      }
      tm1628.clearDot(pDev_cfg->disp_n,0);
    }

    pDev_state->temp_accu += pDev_state->temp_inst;
    pDev_state->temp_avg_ctr++;

    if (pDev_state->temp_avg_ctr == (uint16_t) (pDev_cfg->temp_averages.value)) {
      pDev_state->temp_average_previous = pDev_state->temp_average;
      pDev_state->temp_average = pDev_state->temp_accu / pDev_cfg->temp_averages.value;
      pDev_state->temp_accu = 0;
      pDev_state->temp_avg_ctr = 0;
    }
    // fan/cradle handling
    if (pDev_cfg->dev_type == DEV_HA) { // HA only
      if (REEDSW_OPEN) {
        FAN_ON;
      } else if (pDev_state->temp_average >= FAN_ON_TEMP) {
        FAN_ON;
      } else if (REEDSW_CLOSED && pDev_cfg->fan_only.value == 1 && (pDev_state->temp_average <= FAN_OFF_TEMP_FANONLY)) {
        FAN_OFF;
      } else if (REEDSW_CLOSED && pDev_cfg->fan_only.value == 0 && (pDev_state->temp_average <= FAN_OFF_TEMP)) {
        FAN_OFF;
      }
    }
    
    // security first!
    if (pDev_state->temp_average >= MAX_TEMP_ERR) {
      // something might have gone terribly wrong
      if (pDev_cfg->dev_type == DEV_HA) {
        HA_HEATER_OFF;
        FAN_ON;
      } else {
        SI_HEATER_OFF;
      }     
      // TODO
      HEATERS_OFF; 
#ifdef USE_WATCHDOG
      watchdog_off();
#endif
#ifdef DEBUG
      if (pDev_cfg->dev_type == DEV_HA) {
        Serial.println("Hot air temperature meas. error!");
      } else {
        Serial.println("Soldering iron temperature meas. error!");
      }       
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
        tm1628.showStr(pDev_cfg->disp_n,"*C");
        delay(1000);
        tm1628.showStr(pDev_cfg->disp_n,"Err");
        delay(2000);
        tm1628.clear(pDev_cfg->disp_n);
        delay(1000);
      }
    }
    
#ifdef DEBUG
    int32_t stop_time = micros();
    if (start_time % 1000 == 0)
    {
      if (pDev_cfg->dev_type == DEV_HA) {
        Serial.print("HA Loop time: ");
      } else {
        Serial.print("SI Loop time: ");
      }   
      Serial.println(stop_time - start_time);
    }
#endif
  }
}

void UI_hndl(void)
{
  static uint8_t sp_mode = 0;
  static uint32_t blink_time = millis();
  static uint8_t blink_state = 0;
  uint8_t HA_start;
  DEV_CFG *pDev_cfg = NULL;

  // Blinking feature
  if (millis() - blink_time > BLINK_CYCLE) {
    blink_time = millis();
    if (++blink_state > BLINK_STATE_MAX) {
      blink_state = 0;
    }
  }

  if (ha_state.enabled && ha_state.test_state == TEST_ALL_OK) {
    HA_start = 1;
  } else {
    HA_start = 0;
  }
  
  if (!HA_start && !si_state.enabled) {
    // Nothing to do
    sp_mode = 0;
    return;
  } else if (!HA_start && sp_mode == DEV_HA) { // Device is disabled now
    sp_mode = 0;
  } else if (!si_state.enabled && sp_mode == DEV_SI) { // Device is disabled now
    sp_mode = 0;
  }
  
  // menu key handling
  if (get_key_event_short(KEY_ENTER)) {
    sp_mode++;
    if (sp_mode == DEV_HA && !HA_start) {
      //HA disabled, skip this state
      sp_mode++;
    } else if (sp_mode == DEV_SI && !si_state.enabled) {
      //SI disabled, skip this state
      sp_mode++;
    }    
    if (sp_mode > DEV_SI) {
      sp_mode = 0;
      eep_save(&ha_cfg.temp_setpoint);
      eep_save(&ha_cfg.fan_only);
      eep_save(&si_cfg.temp_setpoint);
    }
  }
  // Select configuration for sp mode
  if (sp_mode == DEV_HA) {
    pDev_cfg = &ha_cfg;
  } else if (sp_mode == DEV_SI) {
    pDev_cfg = &si_cfg;
  }  
  
  if (sp_mode) {
    if (get_key_event_short(KEY_UP | KEY_DOWN)) {// Fan only mode
      if (sp_mode == DEV_HA) {
        pDev_cfg->fan_only.value ^= 0x01;
      }
    } else if (get_key_event_short(KEY_UP)) {
      if (pDev_cfg->temp_setpoint.value < pDev_cfg->temp_setpoint.value_max) {
        pDev_cfg->temp_setpoint.value++;
      }
    } else if (get_key_event_short(KEY_DOWN)) {
      if (pDev_cfg->temp_setpoint.value > pDev_cfg->temp_setpoint.value_min) {
        pDev_cfg->temp_setpoint.value--;
      }
    } else if (get_key_event_long(KEY_UP)) {
      if (pDev_cfg->temp_setpoint.value < (pDev_cfg->temp_setpoint.value_max - 10)) {
        pDev_cfg->temp_setpoint.value += 10;
      } else {
        pDev_cfg->temp_setpoint.value = pDev_cfg->temp_setpoint.value_max;
      }
  
    } else if (get_key_event_long(KEY_DOWN)) {
  
      if (pDev_cfg->temp_setpoint.value > (pDev_cfg->temp_setpoint.value_min + 10)) {
        pDev_cfg->temp_setpoint.value -= 10;
      } else {
        pDev_cfg->temp_setpoint.value = pDev_cfg->temp_setpoint.value_min;
      }
    } 
    
    // Display
    if (blink_state > 7) {
      if (sp_mode == DEV_HA && pDev_cfg->fan_only.value == 1) {
        tm1628.showStr(pDev_cfg->disp_n,"FAn");
      } else {
        tm1628.clear(pDev_cfg->disp_n);
      }
    } else {
      tm1628.showNum(pDev_cfg->disp_n, pDev_cfg->temp_setpoint.value);  // show temperature setpoint
    }
  } 
  
  //Normal operation display
  if (sp_mode == DEV_HA) {    
    temperature_display(&si_cfg, &si_state, blink_state);
  } else if (sp_mode == DEV_SI) {    
    temperature_display(&ha_cfg, &ha_state, blink_state);
  } else if (!sp_mode) {  
    // Not in sp mode
    temperature_display(&ha_cfg, &ha_state, blink_state);
    temperature_display(&si_cfg, &si_state, blink_state);
    
    // Configuration mode
    if (get_key_event_long(KEY_UP | KEY_DOWN)) {
      config_mode();
    }
  }  
}

void temperature_display(DEV_CFG *pDev_cfg, CNTRL_STATE *pDev_state, uint8_t blink_state)
{
  if (!pDev_state->enabled || pDev_state->test_state != TEST_ALL_OK) {
    return;
  }

  if (pDev_state->temp_average <= SAFE_TO_TOUCH_TEMP) {
    if (pDev_cfg->dev_type == DEV_HA && pDev_cfg->fan_only.value == 1) {
      tm1628.showStr(pDev_cfg->disp_n, "FAn");
    } else {
      tm1628.showStr(pDev_cfg->disp_n, "---");
    }
  } else if (pDev_cfg->dev_type == DEV_HA && pDev_cfg->fan_only.value == 1) {
    if (blink_state < 5) {
      tm1628.showStr(pDev_cfg->disp_n, "FAn");
    } else {
      tm1628.showNum(pDev_cfg->disp_n, pDev_state->temp_average);
    }
  } else if (pDev_cfg->display_adc_raw.value == 1) {
    tm1628.showNum(pDev_cfg->disp_n, pDev_state->adc_raw);
  } else if (abs((int16_t) (pDev_state->temp_average) - (int16_t) (pDev_cfg->temp_setpoint.value)) < TEMP_REACHED_MARGIN) {
    tm1628.showNum(pDev_cfg->disp_n, pDev_cfg->temp_setpoint.value);  // avoid showing insignificant fluctuations on the display (annoying)
  } else {
    tm1628.showNum(pDev_cfg->disp_n, pDev_state->temp_average);
  }
}

void config_mode(void)
{
  uint32_t button_scan_time = 0;
  uint32_t blink_time = millis();
  uint8_t blink_state = 0;
  uint8_t param_num = 0;
  uint8_t mode = MODE_DEV_SEL;  
  uint8_t disp = 0;  
  uint8_t dev_type = DEV_HA;
  uint8_t param_max_num = 0;
  uint8_t HA_start;
  CPARAM ** pSet_order = NULL;
  
  HEATERS_OFF;  // security reasons, delay below!
#ifdef USE_WATCHDOG
  watchdog_off();
#endif

  if (ha_state.enabled && ha_state.test_state == TEST_ALL_OK) {
    HA_start = 1;
  } else {
    HA_start = 0;
  }

  // Check if no device select mode
  if (HA_start && !si_state.enabled) {
    dev_type = DEV_HA;
    mode = MODE_VAR_SW;
    disp = ha_cfg.disp_n;
    param_max_num = NELEMS(ha_set_order);
    pSet_order = (CPARAM **)&ha_set_order;
  } else if (!HA_start && si_state.enabled) {
    dev_type = DEV_SI;
    mode = MODE_VAR_SW;
    disp = si_cfg.disp_n;
    param_max_num = NELEMS(si_set_order);
    pSet_order = (CPARAM **)&si_set_order;
  } 
  
  while(1) 
  {   
    // Blinking feature
    if (millis() - blink_time > BLINK_CYCLE) {
      blink_time = millis();
      if (++blink_state > BLINK_STATE_MAX) {
        blink_state = 0;
      }
    }      
    // Key scanning
    if (millis() - button_scan_time > BUTTON_SCANN_CYCLE)
    {
      key_scan();
      button_scan_time = millis();
    }
    
    // Check SW state
    if (!get_sw_state(HA_SW) && !get_sw_state(SI_SW)) {
      //Nothing to do, exit
      break;
    } else if ((ha_state.enabled == 1) != (get_sw_state(HA_SW) == HA_SW)) {
      // HA state change, exit config mode
      break;
    } else if ((si_state.enabled == 1) != (get_sw_state(SI_SW) == SI_SW)) {
      // SI state change, exit config mode
      break;
    } 
 
    //Configure device
    if (mode == MODE_DEV_SEL) {
      // Device select mode
      if (get_key_event_short(KEY_UP | KEY_DOWN)) { // Exit
        break;
      } else if (get_key_event_short(KEY_ENTER)) {
        mode = MODE_VAR_SW;
        if (dev_type == DEV_HA) {
          disp = ha_cfg.disp_n;
          param_max_num = NELEMS(ha_set_order);
          pSet_order = ha_set_order;
        } else {
          disp = si_cfg.disp_n;
          param_max_num = NELEMS(si_set_order);
          pSet_order = si_set_order;
        }
      } else if (get_key_event_short(KEY_UP)) {
        dev_type++;
        if (dev_type > DEV_SI) {
          dev_type = DEV_HA;
        }    
      } else if (get_key_event_short(KEY_DOWN)) {
        dev_type--;
        if (dev_type < DEV_HA) {
          dev_type = DEV_SI;
        } 
      }
      // Display      
      if (dev_type == DEV_HA) {
        tm1628.showStr(si_cfg.disp_n,"Sol");  // show device name
        if (blink_state > 7) {
          tm1628.clear(ha_cfg.disp_n);
        } else {         
          tm1628.showStr(ha_cfg.disp_n,"Hot");  // show device name
        }   
      } else {
        tm1628.showStr(ha_cfg.disp_n,"Hot");  // show parameter name
        if (blink_state > 7) {
          tm1628.clear(si_cfg.disp_n);
        } else {         
          tm1628.showStr(si_cfg.disp_n,"Sol");  // show parameter name
        }  
      }
    } else if (mode == MODE_VAR_SW) {
      // Variable switching mode
      if (get_key_event_short(KEY_UP | KEY_DOWN)) { // To device select mode or exit
        param_num = 0;
        if (HA_start ^ si_state.enabled) {
          // Only one device active, exit
          break;
        } else {
          mode = MODE_DEV_SEL;
        }
      } else if (get_key_event_short(KEY_ENTER)) {
        mode = MODE_VAL_SET;
      } else if (get_key_event_short(KEY_DOWN)) {
        if (param_num+1 < param_max_num) {
          param_num++;
          tm1628.showStr(disp,pSet_order[param_num]->szName);
        }
      } else if (get_key_event_short(KEY_UP)) {
        if (param_num) {
          param_num--;
          tm1628.showStr(disp,pSet_order[param_num]->szName);
        }
      }   
      // Display
      if (blink_state > 7) {
        tm1628.clear(disp);
      } else {
        tm1628.showStr(disp,pSet_order[param_num]->szName);  // show parameter name
      }       
    } else {
      // Edit value mode     
      if (get_key_event_short(KEY_ENTER)) {
        eep_save(pSet_order[param_num]);
        mode = MODE_VAR_SW;
        tm1628.showNum(disp,pSet_order[param_num]->value);
        delay(1000);
        key_event_clear();
      } if (get_key_event_long(KEY_UP)) {
        if (pSet_order[param_num]->value < pSet_order[param_num]->value_max - 10) {
          pSet_order[param_num]->value += 10;
        }
      } else if (get_key_event_long(KEY_DOWN)) {
        if (pSet_order[param_num]->value > pSet_order[param_num]->value_min + 10) {
          pSet_order[param_num]->value -= 10;
        }
      } else if (get_key_event_short(KEY_UP | KEY_DOWN)) { //Exit without saving
        eep_load(pSet_order[param_num]);
        mode = MODE_VAR_SW;
      }else if (get_key_event_short(KEY_UP)) {
        if (pSet_order[param_num]->value < pSet_order[param_num]->value_max) {
          pSet_order[param_num]->value++;
        }
      } else if (get_key_event_short(KEY_DOWN)) {
        if (pSet_order[param_num]->value > pSet_order[param_num]->value_min) {
          pSet_order[param_num]->value--;
        }
      }
      // Display
      if (blink_state > 7) {
        tm1628.clear(disp);
      } else {
        tm1628.showNum(disp,pSet_order[param_num]->value);  // show parameter value
      }      
    }    
  }
  key_event_clear();

#ifdef USE_WATCHDOG
  watchdog_on();
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
#elif defined(SPEED_SENSE_MOD)
  pinMode(FAN_SPEED_PIN, INPUT);  // set as input
#endif

  analogReference(EXTERNAL);  // use external 3.3V (i.e. from Arduino pin) as ADC reference voltage

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
  
  key_scan();

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
#elif defined(SPEED_SENSE_MOD)
      fan = analogRead(FAN_SPEED_PIN);
      tm1628.showNum(DISP_2,fan);
#endif        //CURRENT_SENSE_MOD
      
    }
  }
  
}

void init_state(CNTRL_STATE *pDev_state) {
  if (pDev_state == NULL) {
    return;
  }
  memset(pDev_state, 0, sizeof(CNTRL_STATE));
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
#elif defined(SPEED_SENSE_MOD)
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
#elif defined(SPEED_SENSE_MOD)
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
#elif defined(SPEED_SENSE_MOD)
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
  si_cfg.display_adc_raw.value = si_cfg.display_adc_raw.value_default;

  eep_save(&si_cfg.p_gain);
  eep_save(&si_cfg.i_gain);
  eep_save(&si_cfg.d_gain);
  eep_save(&si_cfg.i_thresh);
  eep_save(&si_cfg.temp_offset_corr);
  eep_save(&si_cfg.temp_setpoint);
  eep_save(&si_cfg.temp_averages);
  eep_save(&si_cfg.display_adc_raw);
}

// Returns zero if ok
uint8_t HA_test(uint8_t state)
{
  uint8_t result;
  static uint32_t test_start = 0;

  if (millis() - test_start > HA_TEST_CYCLE) {
    test_start = millis();  

    HA_HEATER_OFF;
#if defined(CURRENT_SENSE_MOD) || defined(SPEED_SENSE_MOD)
    if (state & FAN_TEST_MASK != 0x10) {// else skip to fan test
#endif
      result = cradle_fail_check(state);
      if (result != CRADLE_OK) {
#ifdef DEBUG
        Serial.print("HA test state: ");
        Serial.println(state,HEX);
#endif
        return result;
      }
#if defined(CURRENT_SENSE_MOD) || defined(SPEED_SENSE_MOD)
    }
#endif
  
#if defined(CURRENT_SENSE_MOD) || defined(SPEED_SENSE_MOD)
    result = fan_fail_check(state);
    if (result != FAN_OK) {
#ifdef DEBUG
      Serial.print("HA test state: ");
      Serial.println(state,HEX);
#endif
      return result;
    }
#endif

#ifdef DEBUG
    Serial.print("HA test state: ");
    Serial.println(state,HEX);
#endif
    key_event_clear();
    return TEST_ALL_OK;
  } else {
    return state;
  }  
}

uint8_t cradle_fail_check(uint8_t state)
{
  if (state == CRADLE_OK) {
    return state; // Test already ended
  }
  // If the wand is not in the cradle when powered up, go into a safe mode
  // and display an error
  if (REEDSW_CLOSED) {
    state = CRADLE_OK;
    return state;
  } else if (state == TEST_INIT) {
    //Cradle error
#ifdef DEBUG
    Serial.println("Cradle error!");
#endif
    state = CRADLE_FAIL1;
  } 
  
  switch(state)
  {
    case CRADLE_FAIL1: 
      tm1628.showStr(ha_cfg.disp_n,"crA");
      state = CRADLE_FAIL2;
      break;   
    case CRADLE_FAIL2: 
      tm1628.showStr(ha_cfg.disp_n,"dLE");
      state = CRADLE_FAIL3;
      break;
    case CRADLE_FAIL3:
      tm1628.clear(ha_cfg.disp_n);
#ifdef DEBUG
      // Skip safe mode in debug!
      state = CRADLE_OK;
      return state;
#endif
      state = CRADLE_FAIL1;
      break;
  }
  
  return state;
}

#if defined(CURRENT_SENSE_MOD) || defined(SPEED_SENSE_MOD)
uint8_t fan_fail_check(uint8_t state) 
{  
  uint8_t fan_current;
  if (state == FAN_OK) {
    return state; // Test already ended
  }
  
  switch(state)
  {
    case TEST_INIT: 
      FAN_ON;
      state = FAN_TEST1;
      tm1628.setDot(ha_cfg.disp_n,2);
      break;   
    case FAN_TEST1: 
      state = FAN_TEST2;
      tm1628.setDot(ha_cfg.disp_n,1);
      break; 
    case FAN_TEST2: 
      state = FAN_TEST3;
      tm1628.setDot(ha_cfg.disp_n,0);
      break; 
    case FAN_TEST3: 
      tm1628.clear(ha_cfg.disp_n);
#ifdef CURRENT_SENSE_MOD
      fan_current = analogRead(FAN_CURRENT_PIN);
      FAN_OFF;
      if ((fan_current < (uint16_t) (ha_cfg.fan_current_min.value)) 
          || (fan_current > (uint16_t) (ha_cfg.fan_current_max.value))) {
        // FAN fail !
        state = FAN_FAIL1;        
#ifdef DEBUG
        Serial.println("Fan current meas. error!");
#endif
#elif defined(SPEED_SENSE_MOD)
      fan_current = analogRead(FAN_SPEED_PIN);
      FAN_OFF;
      if ((fan_current < (uint16_t) (ha_cfg.fan_speed_min.value)) 
          || (fan_current > (uint16_t) (ha_cfg.fan_speed_max.value))) {
        // FAN fail !
        state = FAN_FAIL1;        
#ifdef DEBUG
        Serial.println("Fan speed meas. error!");
#endif
#endif
      } else {
        state = FAN_OK;
      }
      break;
    case FAN_FAIL1:     
      tm1628.showStr(ha_cfg.disp_n,"FAn");
      state = FAN_FAIL2;
      break;         
    case FAN_FAIL2: 
#ifdef CURRENT_SENSE_MOD
      tm1628.showStr(ha_cfg.disp_n,"cur");
#elif defined(SPEED_SENSE_MOD)
      tm1628.showStr(ha_cfg.disp_n,"SPd");
#endif
      state = FAN_FAIL3;
      break;
    case FAN_FAIL3:
      tm1628.clear(ha_cfg.disp_n);
#ifdef DEBUG
      // Skip safe mode in debug!
      state = FAN_OK;
      return state;
#endif
      state = FAN_FAIL1;
      break;
  }
  
  return state;
}
#endif

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

void key_scan(void)
{  
  static uint32_t long_press_scan_time = millis();  
  static uint8_t old_key_state_lscan = 0;
  static uint8_t key_state_ldetected = 0;
  uint8_t tmp_state = 0;
  // For debouncing on/off SW
  static uint8_t sw_state_bounce = 0;

  //SW handling
  if (HA_SW_ON) {
    if (sw_state_bounce & HA_SW) { // State on
      tmp_state |= HA_SW;
    }
    sw_state_bounce |= HA_SW;
  } else {
    if (!(sw_state_bounce & HA_SW)) { // State off
      tmp_state &= ~HA_SW;
    }
    sw_state_bounce &= ~HA_SW;
  }
  if (SI_SW_ON) {
    if (sw_state_bounce & SI_SW) { // State on
      tmp_state |= SI_SW;
    }
    sw_state_bounce |= SI_SW;
  } else {
    if (!(sw_state_bounce & SI_SW)) { // State off
      tmp_state &= ~SI_SW;
    }
    sw_state_bounce &= ~SI_SW;
  }    
#ifdef DEBUG
  if (sw_state ^ tmp_state) {
    Serial.print("SW = 0x");
    Serial.println(tmp_state,HEX);
  }
#endif
  sw_state = tmp_state;

  tmp_state = tm1628.getButtons();
#ifdef DEBUG
  if (key_state ^ tmp_state) {
    Serial.print("BT = 0x");
    Serial.println(tmp_state,HEX);
  }
#endif

  // Key handling
  key_state_s |= (key_state & ~tmp_state) & ~old_key_state_lscan;
  key_state = tmp_state; 
  
  if (millis() - long_press_scan_time > LONG_PRESS_SCANN_CYCLE) 
  {
    long_press_scan_time = millis();    
    key_state_s |= (old_key_state_lscan & ~key_state) & ~key_state_ldetected;
    key_state_ldetected &= ~(old_key_state_lscan & ~key_state); // clear this long press if ended
    key_state_l |= (old_key_state_lscan & key_state); // new key state for long press
    key_state_ldetected |= (old_key_state_lscan & key_state); // save this long press
    old_key_state_lscan = key_state;
  }
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_sw_state(uint8_t sw_mask)
{
  sw_mask &= sw_state;
  return sw_mask;
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
uint8_t get_key_event_short(uint8_t key_mask)
{
  if ((key_state_s & key_mask) == key_mask) {
    key_state_s &= ~key_mask;
    return key_mask;
  }
  return 0;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_event_long(uint8_t key_mask)
{
  if ((key_state_l & key_mask) == key_mask) {
    key_state_l &= ~key_mask;
    return key_mask;
  }
  return 0;
}

///////////////////////////////////////////////////////////////////
//
void key_event_clear(void)
{
  key_state_l = 0;
  key_state_s = 0;
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
