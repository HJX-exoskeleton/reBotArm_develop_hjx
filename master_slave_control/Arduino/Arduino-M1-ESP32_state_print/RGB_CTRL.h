
// WS2812:
#include <Adafruit_NeoPixel.h>

int RAINBOW_STATUS = 0;

Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUMPIXELS, RGB_LED, NEO_GRB + NEO_KHZ800);

void InitRGB(){
  matrix.setBrightness(BRIGHTNESS);
  matrix.begin();
  matrix.show();
}

void colorWipe(uint32_t c) 
{
  for(uint16_t i=0; i<matrix.numPixels(); i++) {
    matrix.setPixelColor(i, c);
  }
  matrix.show();
}


void RGBALLoff(){
  colorWipe(matrix.Color(0, 0, 0));
}


void setSingleLED(uint16_t LEDnum, uint32_t c)
{
  matrix.setPixelColor(LEDnum, c);
  matrix.show();
}


void RGBcolor(byte Rinput, byte Ginput, byte Binput){
  colorWipe(matrix.Color(Rinput, Ginput, Binput));
}