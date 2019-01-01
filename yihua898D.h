#ifndef YIHUA898D_h
#define YIHUA898D_h

/*
 * See the Docs folder for how to add a 1 Ohm current sense
 * resistor to meaure the fan-current.
 *
 * Some time in the future, this may be used to check for
 * true fan-speed via the commutation signal.
 *
 * This only requires an opamp (used as comparator) +
 * a few resistors / caps.
 *
 * http://dangerousprototypes.com/2014/03/01/app-note-fan-health-monitoring-and-the-mic502/
 * http://www.micrel.com/_PDF/App-Notes/an-34.pdf
 *
 */

#include "Arduino.h"
#include "TM1628.h"

 
//#define DEBUG

//#define CURRENT_SENSE_MOD
//#define SPEED_SENSE_MOD

#define USE_WATCHDOG
//#define WATCHDOG_TEST

//TM1628 pins
#define DATA_PIN  8
#define SCLK_PIN  9
#define STB_PIN   7

#define FAN_PIN   4 
#define FAN_INIT  pinMode(FAN_PIN, OUTPUT)
#define FAN_OFF   digitalWrite(FAN_PIN, HIGH)
#define FAN_ON    digitalWrite(FAN_PIN, LOW)

// THIS IS WHERE IT GETS DANGEROUS
// YOU CAN START A FIRE AND DO A LOT OF HARM WITH
// THE HEATER / TRIAC COMMANDS
#define HA_HEATER_PIN     2
#define HA_HEATER_INIT    pinMode(HA_HEATER_PIN, OUTPUT)
#define HA_HEATER_ON      digitalWrite(HA_HEATER_PIN, LOW)
#define HA_HEATER_OFF     digitalWrite(HA_HEATER_PIN, HIGH)

#define SI_HEATER_PIN     3
#define SI_HEATER_INIT    pinMode(SI_HEATER_PIN, OUTPUT)
#define SI_HEATER_ON      digitalWrite(SI_HEATER_PIN, HIGH)
#define SI_HEATER_OFF     digitalWrite(SI_HEATER_PIN, LOW)

#define HEATERS_OFF       HA_HEATER_OFF; SI_HEATER_OFF;


#define HA_TEMP_PIN       A0
#define SI_TEMP_PIN       A1

#ifdef CURRENT_SENSE_MOD
#define FAN_CURRENT_PIN   A2
#elif defined(SPEED_SENSE_MOD)
#define FAN_SPEED_PIN     A2
#endif

#define REEDSW_PIN        10
#define REEDSW_INIT       pinMode(REEDSW_PIN, INPUT_PULLUP)
#define REEDSW_CLOSED     (!digitalRead(REEDSW_PIN))
#define REEDSW_OPEN       digitalRead(REEDSW_PIN)

#define HA_SW_PIN        5
#define HA_SW_INIT       pinMode(HA_SW_PIN, INPUT_PULLUP)
#define HA_SW_ON         (!digitalRead(HA_SW_PIN))
#define HA_SW_OFF        digitalRead(HA_SW_PIN)
// Switch mask in sw state
#define HA_SW            0x01

#define SI_SW_PIN        6
#define SI_SW_INIT       pinMode(SI_SW_PIN, INPUT_PULLUP)
#define SI_SW_ON         (!digitalRead(SI_SW_PIN))
#define SI_SW_OFF        digitalRead(SI_SW_PIN)
// Switch mask in sw state
#define SI_SW            0x02

#define LED_POW   2
#define BLINK_CYCLE       75
#define BLINK_STATE_MAX   10

#define PWM_CYCLES 512L
#define HEATER_DUTY_CYCLE_MAX PWM_CYCLES

#define P_GAIN_DEFAULT_HA 750
#define P_SCALING_DEFAULT_HA (-2)
#define I_GAIN_DEFAULT_HA 40
#define I_SCALING_DEFAULT_HA (-5)
#define D_GAIN_DEFAULT_HA 80
#define D_SCALING_DEFAULT_HA (-1)
#define I_THRESH_DEFAULT_HA 80

#define P_GAIN_DEFAULT_SI 100
#define P_SCALING_DEFAULT_SI 3
#define I_GAIN_DEFAULT_SI 10
#define I_SCALING_DEFAULT_SI (-2)
#define D_GAIN_DEFAULT_SI 0
#define D_SCALING_DEFAULT_SI 0
#define I_THRESH_DEFAULT_SI 80

// Set zero temp if adc is below this value
#define ADC_TEMP_ZERO   25

#define TEMP_GAIN_INT_CORR_DEFAULT_HA 0
#define TEMP_GAIN_DEC_CORR_DEFAULT_HA 559
#define TEMP_OFFSET_CORR_DEFAULT_HA   55

#define TEMP_GAIN_INT_CORR_DEFAULT_SI 0
#define TEMP_GAIN_DEC_CORR_DEFAULT_SI 495
#define TEMP_OFFSET_CORR_DEFAULT_SI   76

#define TEMP_SETPOINT_DEFAULT      80

#define TEMP_AVERAGES_DEFAULT 250L
#define TEMP_REACHED_MARGIN 3

#define MAX_TEMP_ERR 550L
#define DISPLAY_ON_TEMP 75
#define DISPLAY_OFF_TEMP 70

#define FAN_OFF_TEMP 45
#define FAN_ON_TEMP 75

//
// Comment out the following 2 #defines, if you want to use the FAN-speed mod (HW changes required)
// Continue reading below...
//

#define FAN_SPEED_MIN_DEFAULT 150UL
#define FAN_SPEED_MAX_DEFAULT 360UL

// 
// Good starting values with BLDC FAN-speed mod
//
// #define FAN_SPEED_MIN_DEFAULT 450UL
// #define FAN_SPEED_MAX_DEFAULT 800UL
//
// --> Don't forget to extend the ranges in the .ino <--
//
// CPARAM fan_speed_min = { 0, 999, FAN_SPEED_MIN_DEFAULT, FAN_SPEED_MIN_DEFAULT, 18, 19 };
// CPARAM fan_speed_max = { 0, 999, FAN_SPEED_MAX_DEFAULT, FAN_SPEED_MAX_DEFAULT, 20, 21 };
//

#define FAN_CURRENT_MIN_DEFAULT 30UL
#define FAN_CURRENT_MAX_DEFAULT 71UL

#define SLP_TIMEOUT_DEFAULT 30

#define BUTTON_SCANN_CYCLE        100
#define LONG_PRESS_SCANN_CYCLE    300

#define KEY_UP          BT_UP
#define KEY_DOWN        BT_DW
#define KEY_ENTER       BT_EN

#define MODE_DEV_SEL    0
#define MODE_VAR_SW     1
#define MODE_VAL_SET    2

#define HA_TEST_CYCLE   1500

// Must be zero!
#define TEST_ALL_OK     0x00
#define TEST_INIT       0x01
#define CRADLE_OK       0x02
#define CRADLE_FAIL1    0x03
#define CRADLE_FAIL2    0x04
#define CRADLE_FAIL3    0x05
#if defined(CURRENT_SENSE_MOD) || defined(SPEED_SENSE_MOD)
#define FAN_TEST_MASK   0xF0
#define FAN_OK          0x10
#define FAN_TEST1       0x11
#define FAN_TEST2       0x12
#define FAN_TEST3       0x13
#define FAN_FAIL1       0x14
#define FAN_FAIL2       0x15
#define FAN_FAIL3       0x16
#endif

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

#define CPARAM_NULL { 0, 0, 0, 0, 0xFF, 0xFF, ""}

#define DEV_HA  1
#define DEV_SI  2

typedef struct CPARAM {
	int16_t value_min;
	int16_t value_max;
	int16_t value_default;
	int16_t value;
	uint8_t eep_addr_high;
	uint8_t eep_addr_low;
  char szName[4];
} CPARAM;

// HOT AIR/SOLDERING IRON configuration
typedef struct DEV_CFG {
  uint8_t dev_type;
  uint8_t disp_n;
  CPARAM p_gain;
  CPARAM p_scal;
  CPARAM i_gain;
  CPARAM i_scal;
  CPARAM d_gain;
  CPARAM d_scal;
  CPARAM i_thresh;
  CPARAM temp_gain_int_corr;
  CPARAM temp_gain_dec_corr;
  CPARAM temp_offset_corr;
  CPARAM temp_averages;
  CPARAM slp_timeout;
  CPARAM display_adc_raw;
#ifdef CURRENT_SENSE_MOD
  CPARAM fan_current_min;
  CPARAM fan_current_max;
#elif defined(SPEED_SENSE_MOD)
  //
  // See 'FAN-speed mod' (HW changes required)
  // The following 2 CPARAM lines need changes in that case
  //
  CPARAM fan_speed_min;
  CPARAM fan_speed_max;
#endif
  // Not configurable in setting change mode
  CPARAM temp_setpoint;
  CPARAM fan_only;
} DEV_CFG;

// State of the device (HA/SI)
typedef struct CNTRL_STATE {
  int16_t temp_inst;
  int32_t temp_accu;
  int16_t temp_average;
  int16_t temp_average_previous;
  uint16_t temp_avg_ctr;
  
  int16_t heater_ctr;
  int16_t heater_duty_cycle;
  int16_t error;
  int32_t error_accu;
  int16_t velocity;
  float PID_drive;
  
  uint32_t heater_start_time;
  
  uint16_t adc_raw;

  uint8_t test_state;

  uint8_t enabled;

  uint8_t temp_disp_on;
} CNTRL_STATE;

void dev_cntrl(DEV_CFG *pDev_cfg, CNTRL_STATE *pDev_state);
void UI_hndl(void);
void config_mode(void);
void temperature_display(DEV_CFG *pDev_cfg, CNTRL_STATE *pDev_state, uint8_t blink_state);
void eep_load(CPARAM * param);
void eep_save(CPARAM * param);
uint8_t HA_test(uint8_t state);
uint8_t cradle_fail_check(uint8_t state);
#if defined(CURRENT_SENSE_MOD) || defined(SPEED_SENSE_MOD)
uint8_t fan_fail_check(uint8_t state);
#endif
void restore_default_conf(void);
void setup_HW(void);
void init_state(CNTRL_STATE *pDev_state);
void load_cfg(void);
void show_firmware_version(void);
#ifdef USE_WATCHDOG
void watchdog_off(void);
void watchdog_on(void);
uint8_t watchdog_check(void);
void test_F_CPU_with_watchdog(void);
#endif
void key_scan(void);
uint8_t get_sw_state(uint8_t sw_mask);
uint8_t get_key_state(uint8_t key_mask);
uint8_t get_key_event(uint8_t key_mask);
uint8_t get_key_event_short(uint8_t key_mask);
uint8_t get_key_event_long(uint8_t key_mask);
void key_event_clear(void);

#endif				// YIHUA898D_h
