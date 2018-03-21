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

#define MILLISECOND 24000
#define WAIT_TIME 50 * MILLISECOND

#define WHITE_LED_OFFSET 0
#define BLUE_LED_OFFSET 1
#define GREEN_LED_OFFSET 2
#define RED_LED_OFFSET 3

#define NUM_OF_COLORS 4

// Number of LEDs using on 1 meter 60-LED strip
#define LED_COUNT 60

// Color data. Order is WHITE,GREEN,RED,BLUE; WHITE,GREEN,RED,BLUE;...
uint16_t colorDataLen, numBytes;
uint8_t *pixels; // Convert each bit of 32-bit RGBW color to 4 8-bit instructions.

void setup()
{
    colorDataLen = LED_COUNT + 1;
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
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 2400000, 8);

    SSIEnable(SSI0_BASE);

    // Initialize each LED's color values to zero, and all the bytes of each color to zero
    // Order of Bytes is W-G-R-B; W-G-R-B; etc
    uint16_t LED_COUNTer, byteCounter, pixelsId;
    for(LED_COUNTer = 0; LED_COUNTer < colorDataLen; LED_COUNTer++) {
        for(byteCounter = 0; byteCounter < NUM_OF_COLORS; byteCounter++) {
            pixelsId = LED_COUNTer * NUM_OF_COLORS + byteCounter;
            pixels[pixelsId] = 0x0;
        }
    }
}

// Given a red, green, blue, and white color value, set the pixels at led n to the specified colors
void setPixelRGBWColor(uint16_t n, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
    if(n < LED_COUNT) {
        uint16_t pixelsId = n * NUM_OF_COLORS;
        uint8_t *p = &pixels[pixelsId];

        p[WHITE_LED_OFFSET] = white;
        p[BLUE_LED_OFFSET] = blue;
        p[GREEN_LED_OFFSET] = green;
        p[RED_LED_OFFSET] = red;
    }
}

// Given a RGBW aColor, set the pixels at led n to the specified colors
void setPixelColor(uint16_t n, uint32_t aColor) {
    if(n < LED_COUNT) {
       uint8_t white, blue, green, red;

       uint16_t pixelsId = n * NUM_OF_COLORS;
       uint8_t *p = &pixels[pixelsId];

       white = (uint8_t) aColor;
       blue = (uint8_t) aColor >> 8;
       green = (uint8_t) aColor >> 16;
       red = (uint8_t) aColor >> 24;

       p[WHITE_LED_OFFSET] = white;
       p[BLUE_LED_OFFSET] = blue;
       p[GREEN_LED_OFFSET] = green;
       p[RED_LED_OFFSET] = red;
   }
}

// Turn off all colors on LED strip
void clearPixels(void) {
    memset(pixels, 0, numBytes);
}

// Get the clumped 32-bit RGBW color value at led n
uint32_t getColor(uint16_t n) {
    if (n >= LED_COUNT) {
        return 0;
    }
    uint32_t color = 0;
    uint16_t pixelsId = n * NUM_OF_COLORS;
    uint8_t *p = &pixels[pixelsId];

    color = color | (p[RED_LED_OFFSET] << 24);
    color = color | (p[BLUE_LED_OFFSET] << 16);
    color = color | (p[GREEN_LED_OFFSET] << 8);
    color = color | p[WHITE_LED_OFFSET];

    return color;
}

// Helper Function for color retrieval at led n
uint8_t getSpecificColor(uint16_t n, uint8_t offset) {
    if (n >= LED_COUNT) {
        return 0;
    }
    uint16_t pixelsId = n * NUM_OF_COLORS;
    uint8_t *p = &pixels[pixelsId];

    return (uint8_t) p[offset];
}

// Get the white color at led n in hex
uint8_t getWhiteColor(uint16_t n) {
    return getSpecificColor(n, WHITE_LED_OFFSET);
}

// Get the blue color at led n in hex
uint8_t getBlueColor(uint16_t n) {
    return getSpecificColor(n, BLUE_LED_OFFSET);
}

// Get the green color at led n in hex
uint8_t getGreenColor(uint16_t n) {
    return getSpecificColor(n, GREEN_LED_OFFSET);
}

// Get the red color at led n in hex
uint8_t getRedColor(uint16_t n) {
    return getSpecificColor(n, RED_LED_OFFSET);
}

//TODO (REBECCA): Write the Following Functions for Capacitor:
/*
 * (1) function to read value on the data bus
 * (2) interrupts to change the LED value
 */

void showLEDStrip() {

    // Disable interrupts
   GPIOIntDisable(GPIO_PORTA_BASE, LED_STRIP_PIN);

    // Cycles through Pixels and sets color values on LED Strip
    uint16_t pixelIndex;
    for (pixelIndex = 0; pixelIndex < numBytes; pixelIndex++) {
        uint8_t aPixel = pixels[pixelIndex];
        SSIDataPut(SSI0_BASE, aPixel);
        while(SSIBusy(SSI0_BASE)){}
    }

   GPIOIntEnable(GPIO_PORTA_BASE, LED_STRIP_PIN);
}

int main(void) {

    setup();
    //setPixelRGBWColor(0, 255, 255, 255, 255);
    //setPixelRGBWColor(1, 0, 0, 255, 0);
    //setPixelRGBWColor(6, 0, 255, 0, 0);
    //setPixelRGBWColor(3, 0, 0, 0, 0);
   // clearPixels();
    setPixelColor(10, 0xFFFF);
    //setPixelColor(2, 0x0F00);
    while(1)
    {
      showLEDStrip();
      SysCtlDelay(WAIT_TIME);
    }
}
