#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#define LED_STRIP_PIN GPIO_PIN_5
#define SSI_PINS GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5

#define WHITE_LED_OFFSET 0
#define BLUE_LED_OFFSET 1
#define GREEN_LED_OFFSET 2
#define RED_LED_OFFSET 3

#define NUM_OF_COLORS 4

// Number of LEDs using on 1 meter 60-LED strip
uint16_t ledCount = 12;

// Color data. Order is WHITE,GREEN,RED,BLUE; WHITE,GREEN,RED,BLUE;...
uint16_t colorDataLen, numBytes;
uint8_t *pixels; // Convert each bit of 32-bit RGBW color to 4 8-bit instructions.

void setup()
{
    colorDataLen = ledCount + 1;
    numBytes = colorDataLen * NUM_OF_COLORS;
    pixels = (uint8_t*) calloc( numBytes, sizeof(uint8_t) ); // bytes separated by WRGB of each LED

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, SSI_PINS);

    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);

    // The actual desired bit rate, 2400000, produces weird color artifacts. :P 2150000 seems OK.
    // SSI_FRF_MOTO_MODE_0 breaks the timing when the first LED is lit.
    // (Acts as if an extra 3-bit instruction is added to the beginning of the stream.)
    // Inverting the clock phase (SSI_FRF_MOTO_MODE_1) magically fixes the problem. Hardware bug?
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 2150000, 8);

    SSIEnable(SSI0_BASE);

    // Initialize each LED's color values to zero, and all the bytes of each color to zero
    // Order of Bytes is W-G-R-B; W-G-R-B; etc
    uint16_t ledCounter, byteCounter, pixelsId;
    for(ledCounter = 0; ledCounter < colorDataLen; ledCounter++) {
        for(byteCounter = 0; byteCounter < NUM_OF_COLORS; byteCounter++) {
            pixelsId = ledCounter * NUM_OF_COLORS + byteCounter;
            pixels[pixelsId] = 0x0;
        }
    }
}

void setPixelRGBWColor(uint16_t n, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    uint16_t pixelsId;
    uint8_t *p;

    if(n < ledCount) {
        pixelsId = n * NUM_OF_COLORS;
        p = &pixels[pixelsId];

        p[WHITE_LED_OFFSET] = white;
        p[BLUE_LED_OFFSET] = blue;
        p[GREEN_LED_OFFSET] = green;
        p[RED_LED_OFFSET] = red;
    }
}

// Given a currentColor in RGBW order, set the pixels at led n
void setPixelColor(uint16_t n, uint32_t currentColor) {
    uint16_t pixelsId;
    uint8_t *p, white, blue, green, red;

    if(n < ledCount) {
       pixelsId = n * NUM_OF_COLORS;
       p = &pixels[pixelsId];

       white = (uint8_t) currentColor;
       blue = (uint8_t) currentColor >> 8;
       green = (uint8_t) currentColor >> 16;
       red = (uint8_t) currentColor >> 24;

       p[WHITE_LED_OFFSET] = white;
       p[BLUE_LED_OFFSET] = blue;
       p[GREEN_LED_OFFSET] = green;
       p[RED_LED_OFFSET] = red;
   }
}


//TODO: Write a function for writing to an LED, setting a specific hue color, and reading an LED location
int main(void) {
    uint8_t i, timer;

    setup();
    while(1)
    {
      // Wait until SSI0_BASE is ready to send bytes
      while(SSIBusy( SSI0_BASE ));

      for(i = 0; i < numBytes; i++ ) {
        uint8_t data = pixels[i];

        //unsigned long rxData;
        SSIDataPut( SSI0_BASE, data);
      }

      // Pause between animation frames.
      // WS2812 requires a 50us pause between frames to latch the color data.
      // delay(50) is 50 milliseconds, plenty of time.
      for (timer = 0; timer < 5; timer++){}

    }
}
