/***************************************************
  This is a library for the ST7789 IPS SPI display.

  Written by Ananev Ilya.
 ****************************************************/

#include "ST7789.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

static const int spiClk = 40000000; // 1 MHz
SPIClass * vspi = NULL;
  
static const uint8_t PROGMEM
  cmd_240x240[] = {                 		// Initialization commands for 7789 screens
    10,                       				// 9 commands in list:
    ST7789_SWRESET,   ST_CMD_DELAY,  		// 1: Software reset, no args, w/delay
      150,                     				// 150 ms delay
    ST7789_SLPOUT ,   ST_CMD_DELAY,  		// 2: Out of sleep mode, no args, w/delay
      255,                    				// 255 = 500 ms delay
    ST7789_COLMOD , 1+ST_CMD_DELAY,  		// 3: Set color mode, 1 arg + delay:
      0x55,                   				// 16-bit color
      10,                     				// 10 ms delay
    ST7789_MADCTL , 1,  					// 4: Memory access ctrl (directions), 1 arg:
      0x00,                   				// Row addr/col addr, bottom to top refresh
    ST7789_CASET  , 4,  					// 5: Column addr set, 4 args, no delay:
      0x00, ST7789_240x240_XSTART,          // XSTART = 0
	  (240+ST7789_240x240_XSTART) >> 8,
	  (240+ST7789_240x240_XSTART) & 0xFF,   // XEND = 240
    ST7789_RASET  , 4,  					// 6: Row addr set, 4 args, no delay:
      0x00, ST7789_240x240_YSTART,          // YSTART = 0
      (240+ST7789_240x240_YSTART) >> 8,
	  (240+ST7789_240x240_YSTART) & 0xFF,	// YEND = 240
    ST7789_INVON ,   ST_CMD_DELAY,  		// 7: Inversion ON
      10,
    ST7789_NORON  ,   ST_CMD_DELAY,  		// 8: Normal display on, no args, w/delay
      10,                     				// 10 ms delay
    ST7789_DISPON ,   ST_CMD_DELAY,  		// 9: Main screen turn on, no args, w/delay
    255 };                  				// 255 = 500 ms delay

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

Arduino_ST7789::Arduino_ST7789(int8_t dc, int8_t rst,int8_t sdaf, int8_t clkf) 
  : Adafruit_GFX(ST7789_TFTWIDTH_240, ST7789_TFTHEIGHT_240) {
  _dc   = dc;
  _rst  = rst;
}

inline void Arduino_ST7789::spiwrite(uint8_t c) {
  vspi->transfer(c);  
}


void Arduino_ST7789::spiwrite32(uint32_t c) {
  vspi->write32(c);  
}


void Arduino_ST7789::writecommand(uint8_t c) {
  DC_LOW();
  
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  spiwrite(c);
  vspi->endTransaction();
}

void Arduino_ST7789::writedata(uint8_t c) {
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();
  spiwrite(c);
  vspi->endTransaction();
}

void Arduino_ST7789::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;
  DC_HIGH();
  

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

void Arduino_ST7789::commonInit() {
  _ystart = _xstart = 0;
  _colstart  = _rowstart = 0; 

  pinMode(_dc, OUTPUT);
  //REG_WRITE(GPIO_ENABLE_REG,BIT21);
  
  vspi = new SPIClass(VSPI);
  vspi->begin();

    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(50);
    digitalWrite(_rst, LOW);
    delay(50);
    digitalWrite(_rst, HIGH);
    delay(50);
  
}

void Arduino_ST7789::setRotation(uint8_t m) {

  writecommand(ST7789_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);

     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     writedata(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  case 2:
     writedata(ST7789_MADCTL_RGB);
 
     _xstart = _colstart;
     _ystart = _rowstart;
     break;

   case 3:
     writedata(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  }
}

void Arduino_ST7789::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;
  

  writecommand(ST7789_CASET); // Column addr set
  writedata(x_start >> 8);
  writedata(x_start & 0xFF);     // XSTART 
  writedata(x_end >> 8);
  writedata(x_end & 0xFF);     // XEND

  writecommand(ST7789_RASET); // Row addr set
  writedata(y_start >> 8);
  writedata(y_start & 0xFF);     // YSTART
  writedata(y_end >> 8);
  writedata(y_end & 0xFF);     // YEND

  writecommand(ST7789_RAMWR); // write to RAM
}

void Arduino_ST7789::pushColor(uint16_t color) {
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();

  spiwrite(color >> 8);
  spiwrite(color);
  vspi->endTransaction();
}

void Arduino_ST7789::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();

  spiwrite(color >> 8);
  spiwrite(color);

  vspi->endTransaction();
}

void Arduino_ST7789::drawPixels(uint16_t color) {
  spiwrite(color >> 8);
  spiwrite(color);
}

void Arduino_ST7789::mdrawRGBBitmap(int16_t x, int16_t y,const uint16_t *color, int16_t w, int16_t h) {
  setAddrWindow(x, y, (w+x)-1, (h+y)-1);
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();
  vspi->writePixels(color, w*h *2);
  vspi->endTransaction();
}

void Arduino_ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;
   
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  vspi->endTransaction();

}

void Arduino_ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();

  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }

  vspi->endTransaction();
}

void Arduino_ST7789::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Arduino_ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
  DC_HIGH();
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  vspi->endTransaction();
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Arduino_ST7789::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void Arduino_ST7789::invertDisplay(boolean i) {
  writecommand(i ? ST7789_INVON : ST7789_INVOFF);
}

void Arduino_ST7789::beginTransaction(void) {
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
}
void Arduino_ST7789::writePixels(const uint16_t *color, int16_t w, int16_t h) {
  vspi->writePixels(color, w * h * 2);
}
void Arduino_ST7789::endTransaction(void) {
  vspi->endTransaction();
}
void Arduino_ST7789::DC_HIGH(void) {
  digitalWrite(_dc, HIGH);
  //REG_WRITE(GPIO_OUT_W1TS_REG, BIT21);
}

void Arduino_ST7789::DC_LOW(void) {
  digitalWrite(_dc, LOW);
  //REG_WRITE(GPIO_OUT_W1TC_REG, BIT21);
}

void Arduino_ST7789::init(uint16_t width, uint16_t height) {
  commonInit();

  _colstart = ST7789_240x240_XSTART;
  _rowstart = ST7789_240x240_YSTART;
  _height = 240;
  _width = 240;

  displayInit(cmd_240x240);

  setRotation(2);
}
