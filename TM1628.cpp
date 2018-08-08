#include "Arduino.h"

#include "TM1628.h"
byte _curpos = 0x00;
#define BUFFER_SIZE   6
byte buffer[BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00};
//WEP segment addresses
const byte seg_addr[6]={0x02,0x04,0x06,0x08,0x0A,0x0C};//no. bit of digital segments

TM1628::TM1628(byte _dio_pin, byte _clk_pin, byte _stb_pin) {
  this->_dio_pin = _dio_pin;
  this->_clk_pin = _clk_pin;
  this->_stb_pin = _stb_pin;
  _changed = DISP_NONE;

  pinMode(_dio_pin, OUTPUT);
  pinMode(_clk_pin, OUTPUT);
  pinMode(_stb_pin, OUTPUT);

  digitalWrite(_stb_pin, HIGH);
  digitalWrite(_clk_pin, HIGH);

  sendCommand(0x40);
  sendCommand(0x80);

  digitalWrite(_stb_pin, LOW);
  send(0xC0);
  clear(DISP_ALL);
  digitalWrite(_stb_pin, HIGH);
}

void TM1628::begin(boolean active = true, byte intensity = 1) {
  sendCommand(0x80 | (active ? 8 : 0) | min(7, intensity));
}

void TM1628::update() {
  uint8_t start = 0;
  uint8_t end = BUFFER_SIZE;
  
  switch(_changed)
  {  
    case DISP_1:  
        start = 0;
        end = 3;
        break;
    case DISP_2:  
        start = 3;
        end = 6;
        break;
    case DISP_NONE:
        return;
    case DISP_ALL:   
    default:  
        break;
  }  
  
  for (int i=start; i<end; i++)
  {
    sendData(seg_addr[i], buffer[i]);
  }
  
  _changed = DISP_NONE;
}

void TM1628::clear(byte disp) {
  uint8_t start = 0;
  uint8_t end = BUFFER_SIZE;
  
  switch(disp)
  {  
    case DISP_1:  
        start = 0;
        end = 3;
        break;
    case DISP_2:  
        start = 3;
        end = 6;
        break;
    case DISP_NONE:
        return;
    case DISP_ALL:       
    default:  
        break;
  }  
  
  for (int i=start; i<end; i++)
  {
    buffer[i]=0x00;
  }
  
  _changed |= disp;
  update();
}

void TM1628::flush(byte disp) {
  uint8_t start = 0;
  uint8_t end = BUFFER_SIZE;
  
  switch(disp)
  {  
    case DISP_1:  
        start = 0;
        end = 3;
        break;
    case DISP_2:  
        start = 3;
        end = 6;
        break;
    case DISP_NONE:
        return;
    case DISP_ALL:       
    default:  
        break;
  }  
  
  for (int i=start; i<end; i++)
  {
    buffer[i]=0x00;
  }
  
  _changed |= disp;
}

byte TM1628::getButtons() {
  byte keys = 0;

  digitalWrite(_stb_pin, LOW);
  send(0x42);
  for (int i = 0; i < 5; i++) {
    keys |= receive() << i;
  }
  digitalWrite(_stb_pin, HIGH);

  return keys;
}

void TM1628::showStr(byte disp, const char *pStr) {
  uint8_t sLen = 0;
  
  if(pStr == NULL || disp == DISP_NONE)
  {
    return;
  }  

  flush(disp);
  
  sLen = strlen(pStr);
  if((disp == DISP_1 || disp == DISP_2) && sLen > 3)
  {
    sLen = 3;
  }
  else if(sLen > 6)
  {
    sLen = 6;
  }
  
  for(uint8_t i=0; i<sLen; i++)
  {
    setChar(disp, i, pStr[sLen-1-i]);
  }
  update();
}

void TM1628::showNum(byte disp, int16_t number)
{
  //Range -999(9.9.9) .. 999
  if(disp == DISP_NONE)
  {
    return;
  }  
  
	if (number < 0) {
    setDot(disp,1);
    setDot(disp,2);
		number = -number;
	} else {
		// don't clear dot 0, as this is the special indicator
		clearDot(disp,1);
		clearDot(disp,2);
	}


	setDig(disp,0,(uint8_t) (number % 10));
	number /= 10;
	setDig(disp,1,(uint8_t) (number % 10));
	number /= 10;
	setDig(disp,2,(uint8_t) (number % 10));
	update();
}
/*********** mid level  **********/
void TM1628::sendData(byte addr, byte data) {
  sendCommand(0x44);
  digitalWrite(_stb_pin, LOW);
  send(0xC0 | addr);
  send(data);
  digitalWrite(_stb_pin, HIGH);
}

void TM1628::sendCommand(byte data) {
  digitalWrite(_stb_pin, LOW);
  send(data);
  digitalWrite(_stb_pin, HIGH);
}

void TM1628::setDig(byte disp, byte segNum, byte num) {  
  uint8_t start = 0; 
  
  if(disp == DISP_2)
  {
    start = 3;
  }  
  if(start + segNum >= BUFFER_SIZE || disp == DISP_NONE)
  {
    return;
  }
  
  buffer[start + segNum] = NUMBER_FONT[num] | (buffer[start + segNum] & DOT);
  _changed |= disp;
}

void TM1628::setDot(byte disp, byte segNum) {  
  uint8_t start = 0; 
  
  if(disp == DISP_2)
  {
    start = 3;
  }  
  if(start + segNum >= BUFFER_SIZE || disp == DISP_NONE)
  {
    return;
  }
  
  buffer[start + segNum] |= DOT;
  _changed |= disp;
}

void TM1628::clearDot(byte disp, byte segNum) {  
  uint8_t start = 0; 
  
  if(disp == DISP_2)
  {
    start = 3;
  }  
  if(start + segNum >= BUFFER_SIZE || disp == DISP_NONE)
  {
    return;
  }
  
  buffer[start + segNum] &= ~DOT;
  _changed |= disp;
}

void TM1628::setChar(byte disp, byte segNum, byte chr) {
  uint8_t start = 0;
  
  if(disp == DISP_2)
  {
    start = 3;
  }  
  if(start + segNum >= BUFFER_SIZE || disp == DISP_NONE)
  {
    return;
  }
  
  buffer[start + segNum] = FONT_DEFAULT[chr - 0x20];
  _changed |= disp;
}
/************ low level **********/
void TM1628::send(byte data) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(_clk_pin, LOW);
    digitalWrite(_dio_pin, data & 1 ? HIGH : LOW);
    data >>= 1;
    digitalWrite(_clk_pin, HIGH);
  }
}

byte TM1628::receive() {
  byte temp = 0;

  // Pull-up on
  pinMode(_dio_pin, INPUT);
  digitalWrite(_dio_pin, HIGH);

  for (int i = 0; i < 8; i++) {
    temp >>= 1;

    digitalWrite(_clk_pin, LOW);

    if (digitalRead(_dio_pin)) {
      temp |= 0x80;
    }

    digitalWrite(_clk_pin, HIGH);
  }

  // Pull-up off
  pinMode(_dio_pin, OUTPUT);
  digitalWrite(_dio_pin, LOW);

  return temp;
}

