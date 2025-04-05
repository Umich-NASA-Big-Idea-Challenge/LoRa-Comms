/**
 * Send and receive LoRa-modulation packets with a sequence number, showing RSSI
 * and SNR for received packets on the little display.
 *
 * Note that while this send and received using LoRa modulation, it does not do
 * LoRaWAN. For that, see the LoRaWAN_TTN example.
 *
 * This works on the stick, but the output on the screen gets cut off.
*/



// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
//#define FREQUENCY           866.3       // for Europe
 #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           125.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0

#include "message.pb.h"
#include "helper.h"
#include <pb_encode.h>
#include <pb_decode.h>

uint8_t rxBuffer[256];
size_t rxLength = 0;
volatile bool rxFlag = false;

long counter = 0;
uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;

void rx();

void setup() {
  heltec_setup();
  both.println("\n\nRadio init");
  RADIOLIB_OR_HALT(radio.begin());
  // Set the callback function for received packets
  radio.setDio1Action(rx);
  
  // Set radio parameters
  delay(500);
  both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  delay(500);
  both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  delay(500);
  both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  delay(500);
  both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  delay(500);

  
  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

void loop() {
  heltec_loop();
  
  // Transmit a packet every PAUSE seconds or when the button is pressed
  if (button.isSingleClick()) {
    // In case of button click, tell user to wait
    LoRaMessage message = LoRaMessage_init_zero;
    message.type = 1;
    message.message_id = encodeMessageId(0,1);
    uint8_t path_nodes[3] = {0};
    message.encrypted_path = encodeEncryptedPath(path_nodes,1);

    
    uint8_t buffer[LoRaMessage_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    // Encode the message
    bool status = pb_encode(&stream, LoRaMessage_fields, &message);
    if (!status) {
      Serial.println("Failed to encode message");
      return;
    }
  
    // Get the actual size of the encoded message
    size_t message_length = stream.bytes_written;

    radio.clearDio1Action();
    heltec_led(50); // 50% brightness is plenty for this LED
    tx_time = millis();
    RADIOLIB(radio.transmit(buffer, message_length));
    tx_time = millis() - tx_time;
    heltec_led(0);

    both.printf("msg bytes: %zu\n", message_length);

    
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("OK (%i ms)\n", (int)tx_time);
    } else {
      both.printf("fail (%i)\n", _radiolib_status);
    }
    

    radio.setDio1Action(rx);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }

  // If a packet was received, display it and the RSSI and SNR
  if (rxFlag) {
    rxFlag = false;
    
    // Read the received data into our buffer
    rxLength = radio.getPacketLength(); // Get actual packet length
    int state = radio.readData(rxBuffer, rxLength);
    
    if (state == RADIOLIB_ERR_NONE) {
      both.printf("RX packet (%d bytes)\n", rxLength);
      
      // Try to decode as protobuf message
      LoRaMessage receivedMsg = LoRaMessage_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(rxBuffer, rxLength);
      
      if (pb_decode(&stream, LoRaMessage_fields, &receivedMsg)) {
        // Successfully decoded the message
        both.println("Decoded protobuf message:");
        
        // Extract node ID and sequence from the message ID
        uint8_t nodeId, sequence;
        decodeMessageId(receivedMsg.message_id, &nodeId, &sequence);
        
        both.printf("  Type: %d\n", receivedMsg.type);
        both.printf("  From: Node %d, seq %d\n", nodeId, sequence);
        
        // Display other fields if needed
        both.printf("  Speed: %.1f m/s\n", receivedMsg.goal_dm_speed / 10.0);
          both.printf("  Angular: %d deg/s\n", receivedMsg.degrees_per_sec);
      } else {
        // If decoding fails, show raw data as hex
        both.println("Failed to decode as protobuf. Raw data:");
        for (size_t i = 0; i < rxLength; i++) {
          both.printf("%02X ", rxBuffer[i]);
          if ((i + 1) % 8 == 0) both.println();
        }
        both.println();
      }
    }


    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }
}

// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}
