#pragma once


const int numled = 96;
const int pin = 35;

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);


class Tapeled{
  public:
  Tapeled(){
  }
  void init(){
    leds.begin();
  }

  void colorWipe(int color, int wait) {
    for (int i=0; i < leds.numPixels(); i++) {
      leds.setPixel(i, color);
      // leds.show();
      // delayMicroseconds(wait);
    }
    leds.show();
  }
};
