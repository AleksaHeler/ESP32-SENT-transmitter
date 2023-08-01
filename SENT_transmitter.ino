/*
*   SENT transmitter program for ESP32 Dev Module (Single Edge Nibble Transmission)
* 
*   Using built-in serial monitor for communication, we can set up the program 
*   to send a single frame or to send a frame cyclically with specified pause time.
*
*   A SENT message constructed here is 32 bits long (eight nibbles) and consists of the following components:
*     - four bits (one nibble) of status/communication information
*     - 24 bits of signal data (six nibbles) 
*     - four bits (one nibble) for CRC error detection
* 
*   More information about SENT protocol: https://en.wikipedia.org/wiki/SENT_(protocol)
*
*   Timings of pulses measured by logic analyzer in my case for 30us tick was within 0.4% of specification, and for 3us tick 1.2%
*
*   MIT License
*   Copyright (c) 2023 Aleksa Heler, aleksaheler@gmail.com
*/

// Global configuration defines
#define OUTPUT_PIN                  25   // Be careful which pins are safe to use on ESP32 (must be less than 32)
#define TICK_LENGTH_US              3    // (unit: µs) 3 - 90 µs
#define LOW_PULSE_LENGTH_IN_TICKS   5    // (unit: ticks) the low-period is 5 (or more) ticks in length, but in total low+high periods can't exceed 12 ticks
#define FRAME_SYNC_PULSE_IN_TICKS   56   // (unit: ticks) the sync pulse is 56 ticks long

const uint32_t output_pin_reg_select = ((uint32_t)1 << OUTPUT_PIN);

// Define all possible message types
enum message_type_t {
  not_available = 0,
  single,
  cyclic
};

// Global variables
message_type_t message_type = not_available;
uint16_t cyclic_message_pause_ms = 0;
uint32_t last_cyclic_message_timestamp = 0;
uint32_t nibbles_buffer = 0;
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

  // Is there incoming data on serial port? Then receive all that data and process it
  if ( Serial.available() > 0 ) { 
    handle_serial_input();
  }

  // Take different action based on message type 
  switch(message_type){

    // If single message: send it, and set to unavailable
    case single:
      send_frame();
      message_type = not_available;
      break;

    // For cyclic message: if enough time has passed, send frame
    case cyclic:
      // If difference in timestamp between now and last cyclic message is greater than treshold -> send new message
      timestamp = millis();
      if (timestamp - last_cyclic_message_timestamp > cyclic_message_pause_ms) {
        last_cyclic_message_timestamp = timestamp;
        send_frame();
      }
      break;

    // No action
    default:
      break;
  }

  // Delay for stability
  delay(1);
}


// Display manual on how to use this program
void print_welcome_message() {
  Serial.println("=== SENT transmitter ===");
  Serial.println(" To configure enter integer values separated with spaces: ");
  Serial.println("   (message type) [optional parameters] ");
  Serial.println(" Message types and their optional parameters (input in decimal values): ");
  Serial.println("   0) not available - empty message, no additional parameters ");
  Serial.println("   1) single message - 6 nibbles (0-15) for data and 1 nibble for status/communication information ");
  Serial.println("   2) cyclic message - 6 nibbles (0-15) for data and 1 nibble for status/communication information, pause between frames in milliseconds (16bit int, minimum 1ms)");
  Serial.println(" Example: 2 1 2 3 4 5 6 0 1000 ");
  Serial.println("   cyclic message, data = (1, 2, 3, 4, 5, 6), status 0, every 1000ms");
}


// Receive data from serial interface and assemble the frame completely
void handle_serial_input() {
  uint16_t input;  // Used to fetch serial console input

  // Fetch first input number
  input = Serial.parseInt();

  // Sanitize it
  if ( input > cyclic ){
    Serial.println(" -- Error: unknown message type! -- ");
    Serial.flush();
    return;
  }

  // Store message type
  message_type = (message_type_t)input;

  // Assemble right frame for given type
  switch(message_type){

    // If not available, clear buffer
    case not_available:
      nibbles_buffer = 0;
      break;

    // On single message, add given data/info to the frame
    case single:
      nibbles_buffer = 0;
      receive_data_to_buffer();
      receive_info_to_buffer();
      calculate_frame_checksum();
      break;

    // On cyclic message, add given data/info to the frame and pause time
    case cyclic:
      nibbles_buffer = 0;
      receive_data_to_buffer();
      receive_info_to_buffer();
      calculate_frame_checksum();
      // Store cycle message pause time
      input = Serial.parseInt();
      cyclic_message_pause_ms = (uint16_t)input;
      break;

    default:
      Serial.println(" -- Error: unknown message type! -- ");
      break;
  }

  // Clear serial input buffer
  while ( Serial.available() > 0 ){
    Serial.read();
  }
}


// Go over all data nibbles and add them to buffer
void receive_data_to_buffer() {
  uint16_t input, i;

  for (i = 1; i <= 6; i++){
    input = Serial.parseInt();
    nibbles_buffer |= (input & 0xF) << (4 * i);
  }
}


// Add status/communication information nibble
void receive_info_to_buffer() {
  uint16_t input;
  input = Serial.parseInt();
  nibbles_buffer |= (input & 0xF);
}


// Calculate CRC checksum for current frame
// Source: https://electronics.stackexchange.com/questions/284195/sent-crc-calculation
void calculate_frame_checksum() {
  uint8_t i, calculated_crc = 5; // initialize checksum with seed "0101"
  uint32_t crcData = nibbles_buffer;

  for (i = 0; i < 6; i++)
  {
      uint8_t crc_datum = (crcData >> 24) & 0x0F;

      calculated_crc = crc_lookup_table[calculated_crc];
      calculated_crc = calculated_crc ^ crc_datum;
      crcData <<= 4;
  }

  // One more round with 0 as input
  calculated_crc = crc_lookup_table[calculated_crc];

  nibbles_buffer |= (calculated_crc & 0xF) << 28;
}


void send_frame() {
  uint16_t i, data;
  noInterrupts();

  // Send the sync pulse
  send_pulse(FRAME_SYNC_PULSE_IN_TICKS);

  // Send the whole buffer (status, 6 data nibbles, CRC)
  for (i = 0; i < 8; i++){
    // 4bits at a time, starting at LSB
    data = (nibbles_buffer >> (4 * i)) & 0xF;
    // Pulse length in ticks is always (12 + val)
    send_pulse(data + 12);
  }

  // Send the ending pulse
  send_pulse(12);

  interrupts();
}


// Sends a single pulse to SENT pin (pull LOW for N ticks - usually 5, and rest ticks are HIGH)
inline void send_pulse(uint16_t total_length_ticks){
  GPIO.out_w1tc = output_pin_reg_select;                                                  // Set pin to LOW
  delayMicroseconds(LOW_PULSE_LENGTH_IN_TICKS * TICK_LENGTH_US);                          // Wait for low pulse length
  GPIO.out_w1ts = output_pin_reg_select;                                                  // Set pin to HIGH
  delayMicroseconds((total_length_ticks - LOW_PULSE_LENGTH_IN_TICKS) * TICK_LENGTH_US);   // Wait for rest of the frame sync pulse time (minus low pulse length)
}
