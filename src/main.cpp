#include "Protocentral_ADS1220.h" // ADC chip, https://github.com/Protocentral/Protocentral_ADS1220
#include <SPI.h>

// **** GEN CONSTS **** //
#define PLATE_SER_MSG_LENGTH 22 // Bytes from plate ADC - 5x4 bytes + 2 terminating bytes
#define SER_OUTPUT_MSG_LENGTH 39 // 5x4 plate bytes, 3x4 ground-truth loadcell bytes, 2x2 position bytes, 1 piston byte, 2 terminating bytes

// **** ADC CONSTS **** //
// #define PGA 1                 // Programmable Gain = 1
// #define VREF 2.048            // Internal reference of 2.048V
// #define VFSR VREF/PGA
#define PGA 128                   // Internal gain = 128
#define SENSITIVITY 0.00426       // Ratio of excitation to output at full scale = 4 mV/V
#define FSF 13000*9.81            // Force at full-scale (will result in 4 mV output at 1V excitation)
#define FSR (((long int)1<<23)-1) // Full-scale reading
#define NEWTONS_PER_COUNT FSF/(FSR*PGA*SENSITIVITY)
#define SENSOR_COUNT 4            // Number of actual loadcells
#define SENSOR_BYTES 4            // Number of output bytes per sensor
#define ADC_COUNT 2               // Number of ADC chips attached

// **** MOTOR / AXIS CONSTS **** //
#define X_AXIS 0
#define Y_AXIS 1
#define X_MOTOR_CAN_ID 0x01
#define Y_MOTOR_CAN_ID 0x02
#define X_HOMING_DIR 1
#define Y_HOMING_DIR -1
#define X_HOMING_SPEED 0.5
#define Y_HOMING_SPEED 0.5
#define X_HOME_OFFSET 10
#define Y_HOME_OFFSET 10
#define SER_COMMAND_LENGTH 17   // Length of motion commands from PC - 8 bytes for each axis + 1 byte for piston

// **** PISTON PRESSURE REGULATOR CONSTS **** //
#define PRESSURE_MAX_BARS 10      // Upper limit of requested pressure
#define PRESSURE_MAX_CODE 140     // DAC code corresponding to max output (5V) from the piston pressure regulator circuit 
#define PRESSURE_MIN_CODE 32      // DAC code corresponding to 0V from the pressure regulator circuit (seems to go -ve below this) 

// **** GPIO PIN DEFINITIONS **** //
#define Y_MAX_ENDSTOP_PIN 34 // A2 (A1 Max)
#define Y_MIN_ENDSTOP_PIN 39 // A3 (A1 Min)
#define X_MAX_ENDSTOP_PIN 36 // A4 (A0 Max)
#define X_MIN_ENDSTOP_PIN 4  // A5 (A0 Min)
#define PISTON_PRESSURE_PIN 25 // A1
#define PISTON_DIR_PIN 22 // SCL
#define PISTON_UP_PIN 32 // A7, low = switched on
#define PISTON_DOWN_PIN 14 // A6
const int ADS1220_DRDY_PINS[] = {27, 15}; // ADC
const int ADS1220_CS_PINS[] = {5, 33};

// **** RIG STATE GLOBALS **** //
volatile bool homing = false;
volatile bool running = false; // is a calibration run on the go?
volatile bool commandIncoming = false; // is a control command being received (as opposed to a stop command)?

// **** GENERAL GLOBALS **** //
unsigned long t = 0;
unsigned long t0;
byte output_arr[SER_OUTPUT_MSG_LENGTH];
byte sensors_read = 0;
Protocentral_ADS1220 adcs[ADC_COUNT];
volatile bool drdyIntrFlags[] = { false, false };
int32_t adc_data;
int32_t reading;
uint8_t loadcell;

// Piston position possibilities are:
enum pistonPositions {
  PISTON_UP,
  PISTON_DOWN,
  PISTON_RISING,
  PISTON_LOWERING
};
volatile uint8_t pistonPos = PISTON_UP;

// CAN message types (copied from AK-series motor datasheet)
typedef enum {
  CAN_PACKET_SET_DUTY = 0, //Duty cycle mode
  CAN_PACKET_SET_CURRENT, //Current loop mode
  CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
  CAN_PACKET_SET_RPM, //Velocity mode
  CAN_PACKET_SET_POS, // Position mode
  CAN_PACKET_SET_ORIGIN_HERE, //Set origin mode
  CAN_PACKET_SET_POS_SPD, //Position velocity loop mode
} CAN_PACKET_ID;

// **** BUFFER + NUMERICAL FUNCTIONS **** //
// (copied from AK-series motor datasheet)
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

// Convert a float to an unsigned int, given range and number of bits
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
  float span = x_max - x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  return (int) ((x- x_min)*((float)((1<<bits)-1)/span));
}

// **** PISTON CONTROL FUNCTIONS **** // 
// Set piston pressure regulator between ~0 and ~10 bar (though can't source more than compressor)
// Is 8-bit, with ~32 corresponding to 0 and ~140 to 10 bar - these limits are set as consts above
// 10 should give PRESSURE_MAX_CODE (140)
// 0 should give PRESSURE_MIN_CODE (32)
void setPressure(float pressure){
  int pressureCode = (int)(PRESSURE_MIN_CODE + ((PRESSURE_MAX_CODE - PRESSURE_MIN_CODE) * pressure / PRESSURE_MAX_BARS));
  if (pressureCode < 0) pressureCode = 0;
  if (pressureCode > 255) pressureCode = 255;
  dacWrite(PISTON_PRESSURE_PIN, pressureCode);
}

// TODO: Test; these may need to be swapped (i.e. 1 for lowering, 0 for raising)
void lowerPiston(){
  digitalWrite(PISTON_DIR_PIN, 0);
  setPressure(2);
  pistonPos = PISTON_LOWERING;
}

void raisePiston() {
  digitalWrite(PISTON_DIR_PIN, 1);
  setPressure(2);
  pistonPos = PISTON_RISING;
}

// **** AXIS / MOTOR / CAN FUNCTIONS **** //
// Send CAN command with up to 8 bytes of data
void sendCANCommand(uint8_t id, const uint8_t (&data)[8]) {
  // TODO implement
}

// Get CAN ID for the relevant motor axis
uint8_t getAxisId(uint8_t axis) {
  uint8_t id = 0;
  if (axis == X_AXIS) {
    id = X_MOTOR_CAN_ID;
  } else if (axis == Y_AXIS) {
    id = Y_MOTOR_CAN_ID;
  }
  return id;
}

// Get current axis position
float getAxisPosition(int axis){
  // TODO implement
}

// CAN pre-parser, from AK-series motor datasheet. Can probably be scrapped / integrated into sendCANCommand()
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
  uint8_t i=0;
  if (len > 8) {
    len = 8;
  }
  uint8_t msg[8];
  // CanTxMsg TxMessage;
  // TxMessage.StdId = 0;
  // TxMessage.IDE = CAN_ID_EXT;
  // TxMessage.ExtId = id;
  // TxMessage.RTR = CAN_RTR_DATA;
  // TxMessage.DLC = len;
  for(i=0;i<len;i++) {
    msg[i]=data[i];
  }
  sendCANCommand(id, msg);
}

// Current control mode, from AK-series motor datasheet
void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(controller_id |
  ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

// Set axis origin to current position
void setAxisOrigin(int axis){
  // TODO implement
}

// Move axis to given position at a given velocity (servo mode)
// comm_can_set_pos_spd from AK-series motor datasheet
// assume RPA is acceleration - not sure. Also confirm default
void setAxisPosition(uint8_t axis, float pos, int16_t spd, int16_t RPA = 10000){
  uint8_t id = getAxisId(axis);
  int32_t send_index = 0;
  int16_t send_index1 = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer,spd, & send_index1);
  buffer_append_int16(buffer,RPA, & send_index1);
  comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}

// Move axis at constant velocity (Velocity control mode, in RPM)
// comm_can_set_rpm from AK-series motor datasheet
void setAxisVelocity(uint8_t axis, float rpm){
  uint8_t id = getAxisId(axis);
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// Set axis to home and save home position
void homeAxis(int axis){
  homing = true;
  int ENDSTOP_PIN;
  int HOMING_DIR;
  int HOMING_SPEED;
  int HOME_OFFSET;

  if (axis == X_AXIS){
    ENDSTOP_PIN = X_MIN_ENDSTOP_PIN;
    HOMING_DIR = X_HOMING_DIR;
    HOMING_SPEED = X_HOMING_SPEED;
    HOME_OFFSET = X_HOME_OFFSET;
  } else {
    ENDSTOP_PIN = Y_MIN_ENDSTOP_PIN;
    HOMING_DIR = Y_HOMING_DIR;
    HOMING_SPEED = Y_HOMING_SPEED;
    HOME_OFFSET = Y_HOME_OFFSET;
  }

  // home axis at set speed until endstop is hit
  setAxisVelocity(axis, HOMING_DIR*HOMING_SPEED);
  while (digitalRead(ENDSTOP_PIN) != 0){
    delay(10);
  }
  // stop axis and save new endstop position
  setAxisVelocity(axis, 0);
  float endstop_pos = getAxisPosition(axis);
  // move axis to home (HOMING_POSITION away from endstop)
  setAxisVelocity(axis, -HOMING_DIR*HOMING_SPEED);
  while (getAxisPosition(axis) < endstop_pos + HOME_OFFSET){
    delay(10);
  }
  // stop axis, set new origin position
  setAxisVelocity(axis, 0);
  setAxisOrigin(axis);

  homing = false;
}

// Raise piston then home both axes
void homeAxes(){
  raisePiston();
  homeAxis(X_AXIS);
  homeAxis(Y_AXIS);
}

// Stop both axes
void haltAxes(){
  setAxisVelocity(X_AXIS, 0);
  setAxisVelocity(Y_AXIS, 0);
}

// Move both axes to origin position at homing speed
void axesToOrigin() {
  setAxisPosition(X_AXIS, 0, X_HOMING_SPEED);
  setAxisPosition(Y_AXIS, 0, Y_HOMING_SPEED);
}

// **** ADC CONTROL FUNCTIONS **** //
void startADCs(){
  for (int i=0; i<ADC_COUNT; i++) {
    adcs[i].begin(ADS1220_CS_PINS[i], ADS1220_DRDY_PINS[i]);
    adcs[i].Start_Conv();  // Start continuous conversion mode
  }
  t0 = millis();
}

void stopADCs(){
  for (int i=0; i<ADC_COUNT; i++) {
    adcs[i].Stop_Conv(); // Stop continuous conversion mode / power down ADC
  }
}

// **** GENERAL CONTROL FUNCTIONS **** //
// Stop axes and raise piston
void emergencyStop() {
  haltAxes();
  raisePiston();
}

// **** ENDSTOP INTERRUPT HANDLERS **** //
// Note, homing stuff not implemented
void IRAM_ATTR YMinEndstopHandler(){
  Serial.println("SOS: Y-axis min endstop pressed.");
  if (!homing) {
    emergencyStop();
  }
}

void IRAM_ATTR YMaxEndstopHandler(){
  Serial.println("SOS: Y-axis max endstop pressed.");
  if (!homing) {
    emergencyStop();
  }
}

void IRAM_ATTR XMinEndstopHandler(){
  Serial.println("SOS: X-axis min endstop pressed.");
  if (!homing) {
    emergencyStop();
  }
}

void IRAM_ATTR XMaxEndstopHandler(){
  Serial.println("SOS: X-axis max endstop pressed.");
  if (!homing) {
    emergencyStop();
  }
}

// **** ADC DRDY INTERRUPT HANDLERS **** //
void IRAM_ATTR drdy1InterruptHandler(){
  drdyIntrFlags[0] = true;
}

void IRAM_ATTR drdy2InterruptHandler(){
  drdyIntrFlags[1] = true;
}

// **** PISTON POSITION SENSOR INTERRUPT HANDLERS **** //
void IRAM_ATTR pistonUpInterruptHandler(){
  Serial.println("Piston up.");
  pistonPos = PISTON_UP;
}

void IRAM_ATTR pistonDownInterruptHandler(){
  Serial.println("Piston down.");
  pistonPos = PISTON_DOWN;
}

// **** INITIALISATION ROUTINES **** //
void enableInterruptPins(){
  // ADCs
  attachInterrupt(ADS1220_DRDY_PINS[0], drdy1InterruptHandler, FALLING);
  attachInterrupt(ADS1220_DRDY_PINS[1], drdy2InterruptHandler, FALLING);
  // attachInterrupt(ADS1220_DRDY_PINS[i], []() { drdyIntrFlags[i] = true; }, FALLING);

  // Endstops
  attachInterrupt(Y_MIN_ENDSTOP_PIN, YMinEndstopHandler, FALLING);
  attachInterrupt(Y_MAX_ENDSTOP_PIN, YMaxEndstopHandler, FALLING);
  attachInterrupt(X_MIN_ENDSTOP_PIN, XMinEndstopHandler, FALLING);
  attachInterrupt(X_MAX_ENDSTOP_PIN, XMaxEndstopHandler, FALLING);

  // Piston sensors
  attachInterrupt(PISTON_UP_PIN, pistonUpInterruptHandler, FALLING);
  attachInterrupt(PISTON_DOWN_PIN, pistonDownInterruptHandler, FALLING);
}

void setupPinModes(){
  // ADC pins
  pinMode(ADS1220_DRDY_PINS[0], INPUT_PULLUP);
  pinMode(ADS1220_DRDY_PINS[1], INPUT_PULLUP);
  pinMode(ADS1220_CS_PINS[0], OUTPUT);
  pinMode(ADS1220_CS_PINS[1], OUTPUT);

  // Endstops
  pinMode(Y_MIN_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(X_MIN_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(X_MAX_ENDSTOP_PIN, INPUT_PULLUP);

  // Piston direction valve and position sensors
  pinMode(PISTON_DIR_PIN, OUTPUT);
  pinMode(PISTON_UP_PIN, INPUT_PULLUP);
  pinMode(PISTON_DOWN_PIN, INPUT_PULLUP);
}

void setup()
{
  Serial.begin(500000); // Configure PC (USB) serial comms
  Serial2.begin(500000); // Configure plate ESP32 serial comms
  setupPinModes();
  enableInterruptPins();
}

// **** CAPTURE & CONTROL LOOP **** //
void loop()
{
  // Fetch and enact any control updates from Serial1 (PC)
  if (!running && Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'h') { // home axes
      homeAxes();
    } else if (c == 'i') { // stop axes, raise piston
      emergencyStop();
    } else if (c == 's') { // stop axes, raise piston, stop ADCs
      emergencyStop();
      stopADCs();
    } else if (c == 'p') { // get piston position
      Serial.println(pistonPos);
    } else if (c == 'b') { // begin calibration - start ADCs, all messages are assumed to be movement control commands
      running = true;
      startADCs();
    }
  } else if (Serial.available() > 0 and !commandIncoming) { // first check this isn't a stop request (indicated by first character being 's' or 'e')
    char c = Serial.read();
    if (c == 'e') { // end calibration, return to idle state at origin
      running = false;
      stopADCs();
      haltAxes();
      raisePiston();
      int count = 0;
      while (count < 100) { // wait for piston to rise safely, else just stop
        if (!digitalRead(PISTON_DOWN_PIN)) {
          axesToOrigin();
        }
        delay(10);
        count++;
      }
    } else if (c == 's') { // end calibration and emergency stop, don't go to origin
      running = false;
      stopADCs();
      emergencyStop();
    } else if (c == 'c') { // is a control command transmission starting
      commandIncoming = true;
    }
  } else if (Serial.available() > SER_COMMAND_LENGTH && commandIncoming) { // if a control command has been captured, parse it
    // All control messages with set form:
    // 'c' for command
    // - 8x X-axis control bytes to be passed to motor
    // - 8x Y-axis control bytes to be passed to motor
    // - 1x Piston control byte to be decoded
    
    // TODO Finish implementation
    // Read values from serial into buffer until line ending character
    String serCommandStr = Serial.readStringUntil('\r');
    // uint8_t serCommandBuf[SER_COMMAND_LENGTH];
    // Serial.readBytesUntil(0x13, serCommandBuf, SER_COMMAND_LENGTH); // read until \r
    if (serCommandStr.endsWith("\r\n")) {
        // Remove <CR><LF> from end of string
        serCommandStr.remove(serCommandStr.length() - 2, 2);

        // Process message - check if length matches SER_COMMAND_LENGTH. If not, ignore
        if (serCommandStr.length() == SER_COMMAND_LENGTH)
        {
          // First 8 bytes are CAN command to be passed to X-axis motor - slice then transmit
          uint8_t XMotorCommand[8];
          for (int i = 0; i < 8; i++)
          {
              XMotorCommand[i] = serCommandStr.charAt(i);
          }
          sendCANCommand(X_MOTOR_CAN_ID, XMotorCommand); // need to write CAN sender function

          // Second 8 bytes are CAN command to be passed to Y-axis motor
          uint8_t YMotorCommand[8];
          for (int i = 8; i < 16; i++)
          {
              YMotorCommand[i-8] = serCommandStr.charAt(i);
          }
          sendCANCommand(Y_MOTOR_CAN_ID, XMotorCommand);

          // Last byte is piston command, to be parsed
          // - treat like sort of like 8-bit twos complement - with 7-bit pressure and direction up if >=128 and down if <128
          uint8_t pistonCommand = serCommandStr.charAt(16);
          float pistonPressure = PRESSURE_MAX_BARS * abs(pistonCommand) / 128;
          setPressure(pistonPressure);
          if (pistonCommand < 128) { // go down
            if (pistonPos == PISTON_UP || pistonPos == PISTON_RISING) lowerPiston(); // if up or going up, lower - else leave as-is
          } else {  // go up
            if (pistonPos == PISTON_DOWN || pistonPos == PISTON_LOWERING) raisePiston(); // if down or lowering, raise - else leave as-is
          }
        }
  }



while (Serial.available() > 0)
{
  char received = Serial.read();
  // Add the received character to the message
  inSerial += received;

  // Process message when new line character is received
  if (received == '\n')
  {
      if (inSerial.endsWith("\r\n"))
      {
          // Remove <CR><LF> from end of string
          inSerial.remove(inSerial.length() - 2, 2);

          Serial.print("Arduino Received Command: ");
          // Print message as hexadecimal values
          Serial.print(inSerial);

          // Process message - check if length matches 9 bytes - ID + 8 data. If not, ignore
          if (inSerial.length() == 9)
          {
              // Pull out motor ID
              uint8_t id = inSerial.charAt(0);
              Serial.print("\tID: ");
              Serial.write(id);

              // Parse command string into byte array
              uint8_t cmd_data[8];
              for (int i = 0; i < 8; i++)
              {
                  cmd_data[i] = inSerial.charAt(i + 1);
              }

              // Send command data via CAN
              send_cmd(id, cmd_data);
            }
            else
            {
                Serial.print("\t Command length incorrect: ");
                Serial.println(inSerial.length());  // command length
            }

      inSerial = ""; // Clear received buffer
      }
  }








  // Capture force sensor values and add to output_arr
  for (uint8_t i=0; i<ADC_COUNT; i++) {
    if(drdyIntrFlags[i]){  
      drdyIntrFlags[i] = false;
      loadcell = (i * ADC_COUNT) + adcs[i].CurrentChannel;

      reading = adcs[i].Read_Data_Samples(); 
      adcs[i].cycle_input_channel();

      // make little-endian (LSB first) array of bytes to send over serial
      for (uint8_t chunk=0; chunk<SENSOR_BYTES; chunk++) {
        output_arr[4 * loadcell  + chunk] = reading >> (8*chunk);
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

  // have all the ground truth loadcell readings - get latest plate readings + axis + piston states and pass out
  if (sensors_read >= SENSOR_COUNT) {
    // Discard old plate readings from Serial2 to just get latest
    while (Serial2.available() > 2 * PLATE_SER_MSG_LENGTH) {
      uint8_t plateDiscardBuf[PLATE_SER_MSG_LENGTH];
      Serial2.read(plateDiscardBuf, PLATE_SER_MSG_LENGTH);
    }
    // Get newest plate data packet and add to output_arr
    uint8_t plateDataBuf[PLATE_SER_MSG_LENGTH];
    Serial2.readBytesUntil('\r', plateDataBuf, PLATE_SER_MSG_LENGTH);

    // Get rig position and add to output_arr

    // Get piston state and add to output_arr

    // Write output_arr to Serial to pass back to PC
    Serial.write(output_arr, SER_OUTPUT_MSG_LENGTH-2); // most brief method possible - 4 bytes per sensor
    Serial.write("\r\n"); 
    sensors_read = 0;
  }
}