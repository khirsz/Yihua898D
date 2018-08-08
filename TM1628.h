#ifndef TM1628_h
#define TM1628_h

#include "Arduino.h"

#include "TM16XXFonts.h"
//flags for turn on/off
#define ON true
#define OFF false

// Buttons for WEP
#define BT_DW   0x04
#define BT_EN   0x10
#define BT_UP   0x20

// Split displays to two parts for WEP
#define DISP_NONE   0x00
#define DISP_1      0x01
#define DISP_2      0x02
#define DISP_ALL    0x03


class TM1628
{
  public:
	// init
	TM1628(byte _dio_pin, byte _clk_pin, byte _stb_pin);
	
	void begin(boolean active, byte intensity);
	void update();
	void clear(byte disp);
  void flush(byte disp);
	void setDig(byte disp, byte segNum, byte num);
	void setChar(byte disp, byte segNum, byte chr);
  void setDot(byte disp, byte segNum);
  void clearDot(byte disp, byte segNum);
  void showStr(byte disp, const char *pStr);
  void showNum(byte disp, int16_t number);
	byte getButtons();
  protected:
    byte receive();
    void sendData(byte addr, byte data);
    void sendCommand(byte data);
    void send(byte data);
    
    byte _dio_pin;
    byte _clk_pin;
    byte _stb_pin;
    byte _changed;
};

#endif

