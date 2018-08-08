#ifndef youyoue858d_h
#define youyoue858d_h

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

#define DEBUG

#define CURRENT_SENSE_MOD
//#define SPEED_SENSE_MOD

#define USE_WATCHDOG
//#define WATCHDOG_TEST


#define FAN_OFF ( PORTC |= _BV(PC3) )
#define FAN_ON  ( PORTC &= ~_BV(PC3) )
#define FAN_IS_ON ( !(PINC & _BV(PC3)) )
#define FAN_IS_OFF ( PINC & _BV(PC3) )

// THIS IS WHERE IT GETS DANGEROUS
// YOU CAN START A FIRE AND DO A LOT OF HARM WITH
// THE HEATER / TRIAC COMMANDS
#define TRIAC_ON ( PORTB &= ~_BV(PB1) )
#define HEATER_ON TRIAC_ON
#define TRIAC_OFF ( PORTB |= _BV(PB1) )
#define HEATER_OFF TRIAC_OFF

#define BUTTON_SCANN_CYCLE   100

#define LED_POW   2

#define REEDSW_CLOSED ( !(PINB & _BV(PB4)) )
#define REEDSW_OPEN ( PINB & _BV(PB4) )

#define SHOW_SETPOINT_TIMEOUT 2000L

#define HEATER_DUTY_CYCLE_MAX 512L
#define PWM_CYCLES 512L

#define P_GAIN_DEFAULT 650
#define I_GAIN_DEFAULT 15
#define D_GAIN_DEFAULT 500
#define I_THRESH_DEFAULT 45
#define P_GAIN_SCALING 100.0
#define I_GAIN_SCALING 10000.0
#define D_GAIN_SCALING 25.0

#define TEMP_OFFSET_CORR_DEFAULT 33
#define TEMP_SETPOINT_DEFAULT 75

#define TEMP_AVERAGES_DEFAULT 250L
#define TEMP_REACHED_MARGIN 3

#define MAX_TEMP_ERR 550L
#define SAFE_TO_TOUCH_TEMP 40

#define FAN_OFF_TEMP 45
#define FAN_ON_TEMP 60
#define FAN_OFF_TEMP_FANONLY (SAFE_TO_TOUCH_TEMP - 2)

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

#define SLP_TIMEOUT_DEFAULT 10

#define KEY_UP          BT_UP
#define KEY_DOWN        BT_DW
#define ALL_KEYS        (KEY_DOWN | KEY_UP)

#define REPEAT_MASK     (KEY_DOWN | KEY_UP)
#define REPEAT_START    5	// after 5*100ms = 500ms
#define REPEAT_NEXT     1	// every 1*100ms = 100ms


typedef struct CPARAM {
	int16_t value_min;
	int16_t value_max;
	int16_t value_default;
	int16_t value;
	uint8_t eep_addr_high;
	uint8_t eep_addr_low;
} CPARAM;

// HOT AIR configuration
typedef struct HA_CFG {
  CPARAM p_gain;
  CPARAM i_gain;
  CPARAM d_gain;
  CPARAM i_thresh;
  CPARAM temp_offset_corr;
  CPARAM temp_setpoint;
  CPARAM temp_averages;
  CPARAM slp_timeout;
  CPARAM fan_only;
  CPARAM display_adc_raw;
#ifdef CURRENT_SENSE_MOD
  CPARAM fan_current_min;
  CPARAM fan_current_max;
#else
  //
  // See 'FAN-speed mod' (HW changes required)
  // The following 2 CPARAM lines need changes in that case
  //
  CPARAM fan_speed_min;
  CPARAM fan_speed_max;
#endif
} HA_CFG;

// SOLDERING IRON configuration
typedef struct SI_CFG {
  CPARAM p_gain;
  CPARAM i_gain;
  CPARAM d_gain;
  CPARAM i_thresh;
  CPARAM temp_offset_corr;
  CPARAM temp_setpoint;
  CPARAM temp_averages;
  CPARAM slp_timeout;
  CPARAM display_adc_raw;
} SI_CFG;

// State of the device (HA/SI)
typedef struct CNTRL_STATE {
  int16_t temp_inst;
  int32_t temp_accu;
  int16_t temp_average;
  int16_t temp_average_previous;
  
  int16_t heater_ctr;
  int16_t heater_duty_cycle;
  int16_t error;
  int32_t error_accu;
  int16_t velocity;
  float PID_drive;
  
  uint8_t temp_setpoint_saved;
  int32_t temp_setpoint_saved_time;
  
  uint32_t heater_start_time;
  
  uint16_t adc_raw;
} DEV_STATE;

void change_config_parameter(CPARAM * param, const char *string, uint8_t disp);
void clear_eeprom_saved_dot(uint8_t disp);
void eep_load(CPARAM * param);
void eep_save(CPARAM * param);
void fan_test(void);
int main(void);
void restore_default_conf(void);
void set_eeprom_saved_dot(uint8_t disp);
void setup_HW(void);
void show_firmware_version(void);
#ifdef USE_WATCHDOG
void watchdog_off(void);
void watchdog_on(void);
uint8_t watchdog_check(void);
void test_F_CPU_with_watchdog(void);
#endif
uint8_t get_key_press(uint8_t key_mask);
uint8_t get_key_rpt(uint8_t key_mask);
uint8_t get_key_state(uint8_t key_mask);
uint8_t get_key_short(uint8_t key_mask);
uint8_t get_key_long(uint8_t key_mask);
uint8_t get_key_long_r(uint8_t key_mask);
uint8_t get_key_rpt_l(uint8_t key_mask);
uint8_t get_key_common(uint8_t key_mask);
uint8_t get_key_common_l(uint8_t key_mask);

#endif				// youyoue858d_h
