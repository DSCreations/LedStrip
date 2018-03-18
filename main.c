#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#define LED_STRIP_PIN GPIO_PIN_5
#define SSI_PINS GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5

#define BLUE_LED 4
#define GREEN_LED 2
#define RED_LED 1

#define HIGH 1
#define LOW 0

// Number of LEDs using on 1 meter 60-LED strip
uint16_t ledCount = 12;

// Color data. Order is GREEN,RED,BLUE;GREEN,RED,BLUE;...
uint8_t * rgbBuffer;
uint16_t rgbBufferLen;

// Groovy sine wave animation
uint16_t atLED = 0;
uint8_t color = 0;  // Bitmask: 0x1==red, 0x2==green, 0x4==blue

// Convert each bit of 24-bit RGB color to 24 3-bit instructions.
// Store in baudBuffer, then stream the baudBuffer via SPI.
// Possible optimization: Generate these bytes in realtime during SPI transmission?
uint8_t * baudBuffer;
uint16_t baudBufferLen;

void setup()
{
    rgbBufferLen = (ledCount+1)*3;
    rgbBuffer = (uint8_t*) calloc( rgbBufferLen, sizeof(uint8_t) );
    baudBufferLen = (rgbBufferLen) * 3;  // Every color bit becomes a 3-bit instruction, high [^^_] or low [^__]
    baudBuffer = (uint8_t*) calloc( baudBufferLen, sizeof(uint8_t) );

    //TODO(Rebecca): No idea what this is doing...
    //pinMode(RED_LED, OUTPUT);
    //pinMode(GREEN_LED, OUTPUT);
    //pinMode(BLUE_LED, OUTPUT);

    // Copied these commands from Energia's SPI.cpp library, and modified for custom clock speed:
    unsigned long initialData = 0;

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
}
int main(void)
{
uint16_t led;
uint16_t rgbByteIdx;
int8_t rgbBitIdx;
uint16_t i;

static _Bool pulse = HIGH;
static float wavePhase = 0.0;

setup();
while(1)
{
    /*for(led = 0; led < ledCount; led++)
    {
        wavePhase += (0.02 / 6.0);  // Sine wave movement

            uint16_t offset = led*3;
            uint8_t bright = (uint8_t)((sin(led*0.3+wavePhase)+1.0)*0x7f);
            uint8_t clr = (((led < atLED) ? (color+1) : (color)) % 7) + 1;
            rgbBuffer[offset] = (clr&0x2) ? bright : 0;
            rgbBuffer[offset+1] = (clr&0x1) ? bright : 0;
            rgbBuffer[offset+2] = (clr&0x4) ? bright : 0;
     }

    // Convert 8-bit RGB color to 3-baud instructions.
      // The WS2812 chips expect to receive pulse widths of 1/3 or 2/3:
      //   0: ^__  (low)
      //   1: ^^_  (HIGH)
      uint16_t baudBufferIdx = 0;
      uint8_t baudByte = 0x0;  // Build an 8-bit byte, bitmasking with 3-bit ^__ and ^^_ instructions

      int8_t leftShift = 8;

      for(rgbByteIdx = 0; rgbByteIdx < rgbBufferLen; rgbByteIdx++) {
        uint8_t rgbByte = rgbBuffer[rgbByteIdx];

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

      // Wait until SSI0_BASE is ready to send bytes
      while(SSIBusy( SSI0_BASE ));
      uint8_t tempBaudByte = 0x0;
      /*uint32_t tempWGRBColor = 0x0;
      tempWGRBColor = tempWGRBColor | (0x00 << 24); //White
      tempWGRBColor = tempWGRBColor | (0xFF << 16); //Green
      tempWGRBColor = tempWGRBColor | (0x00 << 8); //Red
      tempWGRBColor = tempWGRBColor | (0x00); //Blue*/
      tempBaudByte = tempBaudByte | (0b10 << 6);
      tempBaudByte = tempBaudByte | (0b10 << 4);
      tempBaudByte = tempBaudByte | (0b10 << 2);
      tempBaudByte = tempBaudByte | (0b10);
      baudBuffer[0] = tempBaudByte;
      //baudBuffer[0] = tempWGRBColor;
      baudBuffer[1] = tempBaudByte;
      for(i = 0; i < baudBufferLen; i++ ) {
        uint8_t data = baudBuffer[i];

        //unsigned long rxData;
        SSIDataPut( SSI0_BASE, data);
        //while(ROM_SSIBusy( SSI2_BASE ));
        //ROM_SSIDataGet( SSI2_BASE, &rxData);
      }

      // Pause between animation frames.
      // WS2812 requires a 50us pause between frames to latch the color data.
      // delay(50) is 50 milliseconds, plenty of time.
      // delay( 50 );

      /*// Animation: Advance to the next LED
      atLED = (atLED+1) % ledCount;
      if( atLED==0 ) {  // When we wrap around, advance the color
        color = (color+1)%7;
      }*/

      // Blink the onboard LED to prove we're alive
      //pulse = (pulse == HIGH) ? LOW : HIGH;

      //   SSIDataPut(SSI0_BASE, ui32Data);
}

}
