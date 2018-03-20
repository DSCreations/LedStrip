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
uint8_t * colorData;
uint16_t colorDataLen;

// Groovy sine wave animation
uint8_t currentColor = 0x0;  // Bitmask: 0x1==red, 0x2==green, 0x4==blue

// Convert each bit of 24-bit RGB color to 24 3-bit instructions.
// Store in baudBuffer, then stream the baudBuffer via SPI.
// Possible optimization: Generate these bytes in realtime during SPI transmission?
uint8_t * pixels;
uint16_t numBytes;

void setup()
{

    colorDataLen = (ledCount + 1);
    colorData = (uint8_t*) calloc(colorDataLen, sizeof(uint32_t) ); // 32-bit color of each LED
    numBytes = (colorDataLen) * NUM_OF_COLORS;
    pixels = (uint8_t*) calloc( numBytes, sizeof(uint8_t) ); // bytes separated by WRGB of each LED

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, SSI_PINS);

    /*
          Polarity Phase        Mode
           0     0   SSI_FRF_MOTO_MODE_0
           0     1   SSI_FRF_MOTO_MODE_1
           1     0   SSI_FRF_MOTO_MODE_2
           1     1   SSI_FRF_MOTO_MODE_3
    */
    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);

    // The actual desired bit rate, 2400000, produces weird color artifacts. :P 2150000 seems OK.
    // SSI_FRF_MOTO_MODE_0 breaks the timing when the first LED is lit.
    // (Acts as if an extra 3-bit instruction is added to the beginning of the stream.)
    // Inverting the clock phase (SSI_FRF_MOTO_MODE_1) magically fixes the problem. Hardware bug?
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 2150000, 8);

    SSIEnable(SSI0_BASE);

    // Change the clock divider at your own peril :P The bit rate should already be correct.
    //mySPI.setClockDivider(SPI_CLOCK_DIV4);
    //HWREG( SSI2_BASE + SSI_O_CPSR ) = SPI_CLOCK_DIV4;

    // Initialize each LED's color values to zero, and all the bytes of each color to zero
    // Order of Bytes is W-G-R-B; W-G-R-B; etc
    uint16_t ledCounter;
    uint16_t byteCounter;
    for(ledCounter = 0; ledCounter < colorDataLen; ledCounter++) {
        currentColor = 0x0;
        colorData[ledCounter] = 0x0;
        for(byteCounter = 0; byteCounter < NUM_OF_COLORS; byteCounter++) {
            uint16_t pixelsId = ledCounter * NUM_OF_COLORS + byteCounter;
            pixels[pixelsId] = 0x0;
        }
    }
}

//TODO: Write a function for writing to an LED, setting a specific hue color, and reading an LED location
int main(void)
{
uint16_t ledCounter;
uint16_t byteCounter;
uint16_t i;
uint16_t aColor;


setup();
while(1)
{
    // Convert the values in the colorData array to pixels array, and update the pixels array
    for( ledCounter = 0; ledCounter < colorDataLen; ledCounter++) {
        currentColor = colorData[ledCounter];
        for(byteCounter = 0; byteCounter < NUM_OF_COLORS; byteCounter++) {
            uint16_t pixelsId = ledCounter * NUM_OF_COLORS + byteCounter;
            aColor = -1; // If see this in the code debugging, not going through the switch case
            switch(byteCounter) {
                case 0: // White
                    aColor = (uint8_t) currentColor;
                    break;
                case 1: // Blue
                    aColor = (uint8_t) currentColor >> 8;
                    break;
                case 2: // Green
                    aColor = (uint8_t) currentColor >> 16;
                    break;
                case 3: // Red
                    aColor = (uint8_t) currentColor >> 24;
                    break;
            }
            pixels[pixelsId] = aColor;
        }
    }
    /*for(led = 0; led < ledCount; led++)
    {
        wavePhase += (0.02 / 6.0);  // Sine wave movement

        uint16_t offset = led*4;
        uint8_t bright = (uint8_t)((sin(led*0.3 + wavePhase) + 1.0)*0x7f);
        uint8_t clr = (((led < atLED) ? (color + 1) : (color)) % 7) + 1;
        rgbwBuffer[offset] = (clr&0x1) ? bright : 0;
        rgbwBuffer[offset+1] = (clr&0x2) ? bright : 0;
        rgbwBuffer[offset+2] = (clr&0x4) ? bright : 0;
        rgbwBuffer[offset+3] = (clr&0x8) ? bright : 0;
     }*/

    // Convert 8-bit RGB color to 3-baud instructions.
      // The WS2812 chips expect to receive pulse widths of 1/3 or 2/3:
      //   0: ^__  (low)
      //   1: ^^_  (HIGH)
      /*uint16_t baudBufferIdx = 0;
      uint8_t baudByte = 0x0;  // Build an 8-bit byte, bitmasking with 3-bit ^__ and ^^_ instructions

      int8_t leftShift = 8;

      for(rgbByteIdx = 0; rgbByteIdx < rgbwBufferLen; rgbByteIdx++) {
        uint8_t rgbByte = rgbwBuffer[rgbByteIdx];

        for(rgbBitIdx = 7; rgbBitIdx >= 0; rgbBitIdx-- ) {
          // For this bit: If it's a 1, use the pattern ^^_ ... 0 == ^__
          uint8_t instruction = (rgbByte & (0x1<<rgbBitIdx)) ? 0b110 : 0b100;

          // Prepare our next position. Move 3 bits along the serial output.
          leftShift -= 3;

          if( leftShift >= 0 ) {
            baudByte = baudByte | (instruction << leftShift);
          } else {
            baudByte = baudByte | (instruction >> (-leftShift));
          }

          if( leftShift < 0 ) {
            // Store the completed baudByte
            baudBuffer[baudBufferIdx] = baudByte;
            baudBufferIdx++;

            // Start a new baudByte
            baudByte = 0x0;
            leftShift += 8;
            baudByte = baudByte | (instruction << leftShift);
          }

        }
      }*/
        //uint8_t tempBaudByte = 0x0;
          //uint32_t tempWGRBColor = 0x0;
          //tempWGRBColor = tempWGRBColor | (0 << 24); //Red
          //tempWGRBColor = tempWGRBColor | (0 << 16); //Blue
          //tempWGRBColor = tempWGRBColor | (0 << 8); //Green
          //tempWGRBColor = tempWGRBColor | (1); //White
          /*tempBaudByte = tempBaudByte | (0b10 << 6);
          tempBaudByte = tempBaudByte | (0b10 << 4);
          tempBaudByte = tempBaudByte | (0b10 << 2);
          tempBaudByte = tempBaudByte | (0b10);*/
    for(i = 0; i < numBytes; i++) {
        pixels[i] = 0;
    }
          pixels[0] = 0; // White
          pixels[1] = 0; // Green
          pixels[2] = 255; // Blue
          pixels[3] = 0; // Red
          printf(pixels[2]);
          //baudBuffer[0] = 0x00;
          //baudBuffer[1] = tempBaudByte;

      // Wait until SSI0_BASE is ready to send bytes
      while(SSIBusy( SSI0_BASE ));

      for(i = 0; i < numBytes; i++ ) {
        uint8_t data = pixels[i];

        //unsigned long rxData;
        SSIDataPut( SSI0_BASE, data);
        //while(ROM_SSIBusy( SSI2_BASE ));
        //ROM_SSIDataGet( SSI2_BASE, &rxData);
      }

      // Pause between animation frames.
      // WS2812 requires a 50us pause between frames to latch the color data.
      // delay(50) is 50 milliseconds, plenty of time.
      int timer;
      for (timer = 0; timer < 5; timer++){}

      // Animation: Advance to the next LED
      /*atLED = (atLED+1) % ledCount;
      if( atLED == 0 ) {  // When we wrap around, advance the color
        color = (color+1)%7;
      }*/

      //SSIDataPut(SSI0_BASE, ui32Data);
}

}
