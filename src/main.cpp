//////////////////////////////////////////////////////////////////////////////////////////
//
//    Demo code for the ADS1220 24-bit ADC breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2018 ProtoCentral
//
//    This example gives differential voltage across the AN0 and AN1 pins in mVolts
//
//    Arduino connections:
//
//  |ADS1220 pin label| Pin Function         |Arduino Connection|
//  |-----------------|:--------------------:|-----------------:|
//  | DRDY            | Data ready Output pin|  D02             |
//  | MISO            | Slave Out            |  D12             |
//  | MOSI            | Slave In             |  D11             |
//  | SCLK            | Serial Clock         |  D13             |
//  | CS              | Chip Select          |  D7              |
//  | DVDD            | Digital VDD          |  +5V             |
//  | DGND            | Digital Gnd          |  Gnd             |
//  | AN0-AN3         | Analog Input         |  Analog Input    |
//  | AVDD            | Analog VDD           |  -               |
//  | AGND            | Analog Gnd           |  -               |
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/Protocentral_ADS1220
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include <SPI.h>

// #define PGA 1                 // Programmable Gain = 1
// #define VREF 2.048            // Internal reference of 2.048V
// #define VFSR VREF/PGA
#define PGA 128                   // Internal gain = 128
#define SENSITIVITY 0.00426       // Ratio of excitation to output at full scale = 4 mV/V
#define FSF 13000*9.81            // Force at full-scale (will result in 4 mV output at 1V excitation)
#define FSR (((long int)1<<23)-1) // Full-scale reading
#define NEWTONS_PER_COUNT FSF/(FSR*PGA*SENSITIVITY)
#define SENSOR_COUNT 4
#define SENSOR_BYTES 4            // number of output bytes per sensor
#define ADC_COUNT 2

unsigned long t = 0;
float reads = 0;
unsigned long t0;
byte output_arr[16];
byte sensors_read = 0;
Protocentral_ADS1220 adcs[ADC_COUNT];
const int ADS1220_DRDY_PINS[] = {27, 15};
const int ADS1220_CS_PINS[] = {5, 33};
volatile bool drdyIntrFlags[] = { false, false };

int32_t adc_data;
int32_t reading;
uint8_t sensor;

void IRAM_ATTR drdy1InterruptHndlr(){
  drdyIntrFlags[0] = true;
}

void IRAM_ATTR drdy2InterruptHndlr(){
  drdyIntrFlags[1] = true;
}

void enableInterruptPins(){
  attachInterrupt(ADS1220_DRDY_PINS[0], drdy1InterruptHndlr, FALLING);
  attachInterrupt(ADS1220_DRDY_PINS[1], drdy2InterruptHndlr, FALLING);

  // attachInterrupt(ADS1220_DRDY_PINS[i], []() { drdyIntrFlags[i] = true; }, FALLING);
}

void setup()
{
  Serial.begin(500000);

  for (int i=0; i<ADC_COUNT; i++) {
    adcs[i].begin(ADS1220_CS_PINS[i], ADS1220_DRDY_PINS[i]);
    adcs[i].Start_Conv();  //Start continuous conversion mode
  }

  enableInterruptPins();
  t0 = millis();
}

void loop()
{
  for (uint8_t i=0; i<ADC_COUNT; i++) {
    if(drdyIntrFlags[i]){  
      drdyIntrFlags[i] = false;
      sensor = (i * ADC_COUNT) + adcs[i].CurrentChannel;

      reading = adcs[i].Read_Data_Samples(); 
      adcs[i].cycle_input_channel();

      // make little-endian (LSB first) array of bytes to send over serial
      for (uint8_t chunk=0; chunk<SENSOR_BYTES; chunk++) {
        output_arr[4 * sensor + chunk] = reading >> (8*chunk);
      }

      sensors_read += 1;

      // float force = (float)(reading*NEWTONS_PER_COUNT);           // in N
      // // float reading = (float)(adc_data*VFSR*1000)/FSR;               // in mV
      // Serial.println(String(sensor) + ":" + String(force) + "N");  // directly print float value

      // if (sensors_read >= SENSOR_COUNT) {
      //   sensors_read = 0;
      //   Serial.println("");
      // }
    }
  }

  // have a full packet, pass out
  if (sensors_read >= SENSOR_COUNT) {
    Serial.write(output_arr, 16); // most brief method possible - 4 bytes per sensor
    Serial.write("\r\n"); 
    sensors_read = 0;
  }
}