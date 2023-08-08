/*
*   SENT protocol transmitter program for ESP32 Dev Module (Single Edge Nibble Transmission)
* 
*   Using built-in serial monitor for communication, we can set up the program 
*   to send a single frame or to send a frame cyclically with specified pause time.
*
*   A SENT message constructed here is for example 32 bits long (eight nibbles) and consists of the following components:
*     - 4  bits (one nibble)  of status/communication information
*     - 24 bits (six nibbles) of signal data
*     - 4  bits (one nibble)  for CRC error detection
*   
*   Program logic cycle:
*     - check if new data is available on serial interface, if it is:
*       - fetch data from serial interface
*       - parse that data into buffer
*       - calculate all wait times needed to send pulses for given data
*       - send pulses as fast as possible
*     - if we're in a cyclic message mode, passive wait (without delay) until another transmission is needed
*       - if that time is up, then no need to calculate everything again, just send the pulses as last time 
* 
*   More information about SENT protocol: https://en.wikipedia.org/wiki/SENT_(protocol)
*
*   Build output log (ESP32 Dev Module):
*     Sketch uses 264893 bytes (20%) of program storage space. Maximum is 1310720 bytes.
*     Global variables use 21448 bytes (6%) of dynamic memory, leaving 306232 bytes for local variables. Maximum is 327680 bytes.
*
*   Measurements from example message command using logic analyzer at 100MS/s, tick time 3us (single message): 1 6 15 14 13 12 11 10
*   Rise/fall time of the signal was measured at around 1us, so some error will be caused by that (using dupont wires to connect on the breadboard)
*
*     signal        value  ticks  us expected  us measured  %error
*     ------------------------------------------------------------
*     sync pulse           56     168          169.66       0.99%
*     status        6      18     54           54.09        0.17%
*     data 0        15     27     81           81.15        0.18%
*     data 1        14     26     78           77.75        0.32%
*     data 2        13     25     75           74.99        0.01%
*     data 3        12     24     72           72.25        0.35%
*     data 4        11     23     69           68.95        0.07%
*     data 5        10     22     66           66.2         0.30%
*     CRC           7      19     57           58.49        2.61%
*
*   MIT License
*   Copyright (c) 2023 Aleksa Heler, aleksaheler@gmail.com
*/

// Global configuration defines
#define OUTPUT_PIN                  25   // Be careful which pins are safe to use on ESP32 (must be less than 32)
#define DATA_NIBBLES_COUNT          6    // number of data nibbles in the message (usually 3 or 6)

#define TICK_LENGTH_US              3    // (unit: µs) 3 - 90 µs
#define FRAME_SYNC_PULSE_IN_TICKS   56   // (unit: ticks) the sync pulse is 56 ticks long
#define LOW_PULSE_LENGTH_IN_TICKS   5    // (unit: ticks) the low-period is 5 (or more) ticks in length, but in total low+high periods can't exceed 12 ticks

#define SERIAL_BUFFER_SIZE          64   // size (number of bytes) of the buffer used when receiving raw serial data
#define SERIAL_BUFFER_TOKENS       " "   // characters that will be used to separate input data in serial buffer string

// Constants
const uint32_t output_pin_reg_select = ((uint32_t)1 << OUTPUT_PIN);
const uint32_t output_pin_low_pulse_length_us = LOW_PULSE_LENGTH_IN_TICKS * TICK_LENGTH_US;

// Define all possible message types
enum message_type_t {
  not_available = 0,
  single = 1,
  cyclic = 2
};

// Define everything we need/want to know about a message
// This includes pre-calculated wait times for pulses, so we don't have to calculate while sending the pulses and thus hopefuly being quicker and more accurate
typedef struct {
  message_type_t message_type;                                       // Type of message user has selected
  uint16_t cyclic_message_pause_ms;                                  // How long to wait between messages
  uint32_t last_cyclic_message_timestamp;                            // Timestamp from last nibble transmission
  uint8_t nibbles_buffer[DATA_NIBBLES_COUNT + 2];                    // Nibble values (status, 6 data nibbles, crc) 
  uint16_t output_high_pulse_length_buffer[DATA_NIBBLES_COUNT + 4];  // Time to wait during 'high' part of the pulse for given nibble (8 nibbles total, plus sync pulse, and end pulse)
} SENT_message;

// Global variables
char serial_buffer[SERIAL_BUFFER_SIZE];
SENT_message message;
const uint8_t crc_lookup_table[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5};


// Called one time during boot, sets up everything needed
void setup() {
  // Configure serial connection, used for interacting with user, configuring the message
  Serial.begin(9600);

  // Configure the pin used for SENT to output idle state
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH);

  // Print how to use this program to serial console
  print_welcome_message();
}


// Called continuously while the board is powered
void loop() {
  uint32_t timestamp;

  // Is there incoming data on serial port? Then receive all that data to buffer
  if (Serial.available() > 0) {
    Serial.readString().toCharArray(serial_buffer, SERIAL_BUFFER_SIZE);
  }

  // If we received some data in serial comms
  if (strlen(serial_buffer) !=0){
    // Parse it
    parse_input_data();

    // Calculate all wait times needed to send pulses for given data
    calculate_message_timings();
  }

  // Take different action based on message type 
  switch(message.message_type){

    // If single message: send it, and set to unavailable so we don't send it again
    case single:
      send_frame();
      message.message_type = not_available;
      break;

    // For cyclic message: if enough time has passed, send frame
    case cyclic:
      // If difference in timestamp between now and last cyclic message is greater than treshold -> send new message
      timestamp = millis();
      if (timestamp - message.last_cyclic_message_timestamp > message.cyclic_message_pause_ms) {
        message.last_cyclic_message_timestamp = timestamp;
        send_frame();
      }
      break;

    // No action
    default:
      break;
  }

  // Delay for stability
  delayMicroseconds(200);
}


// Display manual on how to use this program
void print_welcome_message() {
  Serial.println();
  Serial.println();
  Serial.println("===== SENT transmitter =====");
  Serial.println();
  Serial.println(" To configure enter integer values separated with spaces: ");
  Serial.println("   [message type] [optional parameters] ");
  Serial.println();
  Serial.println(" Message types and their optional parameters (input in base 10 integer values): ");
  Serial.println("   0) not available  - empty message, no additional parameters ");
  Serial.println("   1) single message - 1 nibble for status/communication information, N nibbles (values: 0-15) for data,");
  Serial.println("   2) cyclic message - 1 nibble for status/communication information, N nibbles (values: 0-15) for data, pause between frames in milliseconds (16bit int, minimum 1ms)");
  Serial.println();
  Serial.println(" Examples (with DATA_NIBBLES_COUNT set to 6): ");
  Serial.println("   0                     -> dont send anything (or stop sending previous cyclic message) ");
  Serial.println("   1 6 15 14 13 12 11 10 -> single message, status 6, data = (0xF, 0xE, 0xD, 0xC, 0xB, 0xA) ");
  Serial.println("   2 0 1 2 3 4 5 6 1000  -> cyclic message, status 0, data = (1, 2, 3, 4, 5, 6), every 1000ms ");
  Serial.println();
}


// Receive data from serial interface and assemble the frame completely
void parse_input_data() {
  uint16_t parsedInt = 0, i = 0;
  char* substring = NULL;

  Serial.println();
  Serial.print(" > Serial input: ");
  Serial.println(serial_buffer);

  // Get the first substring from our buffer, using the space character ' ' as delimiter
  substring = strtok(serial_buffer, SERIAL_BUFFER_TOKENS);

  // First is message type
  if (substring == NULL) {
    Serial.println(" > ERROR! problems with parsing input string! Check all the parameters are present!");
    serial_buffer[0] = '\0';
    return;
  }
  parsedInt = atoi(substring);
  if(parsedInt > cyclic){
    Serial.print(" > ERROR! input message type has unexpected value (expected 0, 1 or 2): ");
    Serial.println(parsedInt);
    parsedInt = 0;
  }
  message.message_type = (message_type_t)parsedInt;

  // If message type is not empty: parse other data
  if (message.message_type != not_available) {
    // Every non empty message has status nibble
    substring = strtok(NULL, SERIAL_BUFFER_TOKENS);
    if (substring == NULL) {
      Serial.println(" > ERROR! problems with parsing input string! Check all the parameters are present!");
      serial_buffer[0] = '\0';
      return;
    }
    parsedInt = atoi(substring);
      // Sanitize input
    if (parsedInt > 15) {
      Serial.print(" > ERROR! input status nibble has value greater than allowed (expected 0-15, or 0x0-0xF): ");
      Serial.println(parsedInt);
      parsedInt = 0;
    }
    message.nibbles_buffer[0] = parsedInt;
    
    // Now parse the 6 nibbles
    for (i = 0; i < DATA_NIBBLES_COUNT; i++){
      // Get the next substring and convert to int
      substring = strtok(NULL, SERIAL_BUFFER_TOKENS);
      if (substring == NULL) {
        Serial.println(" > ERROR! problems with parsing input string! Check all the parameters are present!");
        serial_buffer[0] = '\0';
        return;
      }
      parsedInt = atoi(substring);
      // Sanitize input
      if (parsedInt > 15) {
        Serial.print(" > ERROR! input data nibble with index ");
        Serial.print(i);
        Serial.print(" has value greater than allowed (expected 0-15, or 0x0-0xF): ");
        Serial.println(parsedInt);
        parsedInt = 0;
      }
      // Save data to buffer
      message.nibbles_buffer[i + 1] = parsedInt;
    }

    // Calculate CRC nibble
    message.nibbles_buffer[DATA_NIBBLES_COUNT + 1] = calculate_frame_checksum(message.nibbles_buffer);

    // If our message is cyclic
    if (message.message_type == cyclic) {
      // Also parse the cycle time
      substring = strtok(NULL, SERIAL_BUFFER_TOKENS);
      if (substring == NULL) {
        Serial.println(" > ERROR! problems with parsing input string! Check all the parameters are present!");
        serial_buffer[0] = '\0';
        return;
      }
      parsedInt = atoi(substring);
      message.cyclic_message_pause_ms = parsedInt;
    }
    
    display_final_message();
  }
  else {
    Serial.println(" > Stopping message transmission! ");
  }


  // Free the dynamic allocation, we don't need the buffer anymore
  serial_buffer[0] = '\0';
}


// Calculate CRC checksum for current frame
// Source: https://electronics.stackexchange.com/questions/284195/sent-crc-calculation
uint8_t calculate_frame_checksum(uint8_t nibbles_buffer[8]) {
  uint8_t i, calculated_crc = 5; // Initialize checksum with seed "0101"
  uint64_t crcData = 0;

  // Convert array of N nibbles into a max 64bit number (so max 14 data nibbles supported, plus 2 for status and crc)
  for (i = 0; i < DATA_NIBBLES_COUNT + 2; i++){
    crcData |= (nibbles_buffer[i] & 0xF) << (i*4);
  }

  // CRC calculation loop, repeat 6 times
  for (i = 0; i < 6; i++)
  {
    uint8_t crc_datum = (crcData >> 24) & 0x0F;

    calculated_crc = crc_lookup_table[calculated_crc];
    calculated_crc = calculated_crc ^ crc_datum;
    crcData <<= 4;
  }

  // One more round with 0 as input
  calculated_crc = crc_lookup_table[calculated_crc];

  // Return calculated CRC
  return calculated_crc;
}


void calculate_message_timings(){
  uint16_t i;

  // Sync pulse, 56 ticks, where 5 is low, and rest are high, then multiply by tick length time 
  message.output_high_pulse_length_buffer[0] = (FRAME_SYNC_PULSE_IN_TICKS - LOW_PULSE_LENGTH_IN_TICKS) * TICK_LENGTH_US;

  // 8 nibbles: status, 6 data, crc
  for (i = 0; i < DATA_NIBBLES_COUNT+2; i++) {
    message.output_high_pulse_length_buffer[i + 1] = ((message.nibbles_buffer[i] + 12) - LOW_PULSE_LENGTH_IN_TICKS) * TICK_LENGTH_US;
  }

  // End pulse (value 0) so the last nibble doesn't end on high, or else it wouldn't be detected
  message.output_high_pulse_length_buffer[DATA_NIBBLES_COUNT+2+1] = (12 - LOW_PULSE_LENGTH_IN_TICKS) * TICK_LENGTH_US;
}


// Print finally parsed message with calculated CRC to serial monitor
void display_final_message() {
  int i;
  Serial.println(" > Message: ");

  Serial.print("     Status = ");
  Serial.println(message.nibbles_buffer[0]);

  Serial.print("       Data = ");
  for (i = 1; i < DATA_NIBBLES_COUNT+1; i++) {
    Serial.print(message.nibbles_buffer[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("        CRC = ");
  Serial.println(message.nibbles_buffer[DATA_NIBBLES_COUNT+1]);
}


void send_frame() {
  uint16_t i, data;
  
  // Disable interrupts so we don't get interrupted, and do this as fast as possible
  noInterrupts();

  // Send the whole message including buffer at the same time (sync, status, 6 data nibbles, CRC, end pulse)
  for (i = 0; i < DATA_NIBBLES_COUNT+4; i++){
    send_pulse(message.output_high_pulse_length_buffer[i]);
  }

  interrupts();
}


// Sends a single pulse to SENT pin (pull LOW for N ticks - usually 5, and rest ticks are HIGH)
inline void send_pulse(uint16_t high_pulse_length){
  GPIO.out_w1tc = output_pin_reg_select;              // Set pin to LOW
  delayMicroseconds(output_pin_low_pulse_length_us);  // Wait for low pulse length
  GPIO.out_w1ts = output_pin_reg_select;              // Set pin to HIGH
  delayMicroseconds(high_pulse_length);               // Wait for rest of the frame sync pulse time
}


// Inline implementation of the delay for x microseconds function, in order to not have the function call routine each time
// Not original code, but taken from: https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/delay.c
inline void delay_microseconds( unsigned int usec )
{
  uint32_t n = usec * (48000000ul / 1000000) / 3;
  __asm__ __volatile__(
    "1:              \n"
    "   sub %0, #1   \n" // substract 1 from %0 (n)
    "   bne 1b       \n" // if result is not 0 jump to 1
    : "+r" (n)           // '%0' is n variable with RW constraints
    :                    // no input
    :                    // no clobber
  );
}
