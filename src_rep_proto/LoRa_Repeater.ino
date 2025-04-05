// Enable power button functionality on Heltec
#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>

// LoRa Configuration
#define FREQUENCY           905.2  // US Frequency
#define BANDWIDTH           125.0  // kHz
#define SPREADING_FACTOR    9      // 5-12 (Higher = Longer Range, Slower)
#define TRANSMIT_POWER      0      // dBm (Range: -9 to 22)

// LBT & Timing Configurations
#define LBT_BACKOFF_DELAY   10     // ms, base backoff when channel is busy
#define MAX_RETRIES         3      // Max retries if channel is busy
#define TRANSMISSION_DELAY  100    // ms, delay between transmissions
#define RX_TASK_DELAY       10     // ms, small delay for task switching

// Protobuf includes
#include "message.pb.h"
#include "helper.h"
#include <pb_encode.h>
#include <pb_decode.h>

// Buffer for received messages
uint8_t rxBuffer[256];
size_t rxLength = 0;

// RTOS Handles
SemaphoreHandle_t xSemaphore;
TaskHandle_t sendTaskHandle;
TaskHandle_t receiveTaskHandle;

// Message queue structure with priority
struct PriorityMessage {
    LoRaMessage message;
    uint8_t priority;  // Lower number = higher priority
};

// Queue for priority-based message handling
QueueHandle_t priorityQueue;

// Volatile flags for ISR
volatile bool isReceiving = false;
volatile bool rxFlag = false;

// Other global variables
bool display_is_on = true;

// Interrupt Service Routine (ISR) for receiving messages
void IRAM_ATTR onReceiveInterrupt() {
    rxFlag = true;
    isReceiving = true;
}

// Function to check if the channel is free
bool isChannelFree() {
    return !isReceiving;
}

// Function to determine message priority based on type
uint8_t getMessagePriority(uint32_t msgType) {
    switch (msgType) {
        case 0:  return 0;  // Highest priority
        case 1:  return 1;
        case 2:  return 2;
        default: return 3;  // Lowest priority
    }
}

// Task: Send Messages
void sendMessage(void *pvParameters) {
    PriorityMessage priorityMsg;
    
    while (true) {
        // Wait for a message in the queue
        if (xQueueReceive(priorityQueue, &priorityMsg, portMAX_DELAY) == pdTRUE) {
            // Create a new message based on the received one
            LoRaMessage outMsg = priorityMsg.message;
            
            // Get the encrypted path and update it
            uint8_t path_nodes[3] = {0};
            uint8_t path_length = 0;
            
            // Decode the current path
            decodeEncryptedPath(outMsg.encrypted_path, path_nodes);
            
            // Add our node ID (0) to the path if there's room
            if (path_length < 3) {
                path_nodes[path_length] = 0;  // Node ID 0 for this repeater
                path_length++;
                outMsg.encrypted_path = encodeEncryptedPath(path_nodes, path_length);
            }
            
            // Wait until the channel is free
            while (!isChannelFree()) {
                both.println("Channel busy, waiting...");
                vTaskDelay(LBT_BACKOFF_DELAY / portTICK_PERIOD_MS);
            }

            // Try to acquire the semaphore for transmission
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                int retries = 0;

                while (retries < MAX_RETRIES) {
                    if (isChannelFree()) {
                        // Encode message for transmission
                        uint8_t buffer[LoRaMessage_size];
                        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
                        
                        bool status = pb_encode(&stream, LoRaMessage_fields, &outMsg);
                        if (!status) {
                            both.println("Failed to encode message");
                            break;
                        }
                        
                        size_t message_length = stream.bytes_written;
                        
                        // Extract node ID and sequence from the message ID
                        uint8_t nodeId, sequence;
                        decodeMessageId(outMsg.message_id, &nodeId, &sequence);
                        
                        both.printf("Relay: Type=%d, Node=%d, Seq=%d (%d bytes) ", 
                                   outMsg.type, nodeId, sequence, message_length);
                        
                        // Transmit the message
                        radio.clearDio1Action();
                        heltec_led(50);
                        RADIOLIB(radio.transmit(buffer, message_length));
                        heltec_led(0);

                        if (_radiolib_status == RADIOLIB_ERR_NONE) {
                            both.printf("Sent\n");
                        } else {
                            both.printf("TX failed (%i)\n", _radiolib_status);
                        }
                        break;  // Exit retry loop if successful
                    } else {
                        // Channel is busy, backoff and retry
                        both.println("Channel busy, retrying...");
                        vTaskDelay((LBT_BACKOFF_DELAY + random(0, 100)) / portTICK_PERIOD_MS);
                        retries++;
                    }
                }
                
                // Re-enable receiving
                radio.setDio1Action(onReceiveInterrupt);
                RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
                
                xSemaphoreGive(xSemaphore);  // Release the semaphore
                vTaskDelay(TRANSMISSION_DELAY / portTICK_PERIOD_MS);
            }
        }
    }
}

// Task: Receive Messages
void receiveMessages(void *pvParameters) {
    while (true) {
        // Attempt to acquire the semaphore for receiving
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            if (rxFlag) {
                rxFlag = false;  // Reset flag
                
                // Read the received data
                rxLength = radio.getPacketLength();
                int state = radio.readData(rxBuffer, rxLength);
                isReceiving = false;  // Mark channel as free

                if (state == RADIOLIB_ERR_NONE) {
                    // Try to decode as protobuf message
                    LoRaMessage receivedMsg = LoRaMessage_init_zero;
                    pb_istream_t stream = pb_istream_from_buffer(rxBuffer, rxLength);
                    
                    if (pb_decode(&stream, LoRaMessage_fields, &receivedMsg)) {
                        // Extract node ID and sequence from the message ID
                        uint8_t nodeId, sequence;
                        decodeMessageId(receivedMsg.message_id, &nodeId, &sequence);
                        
                        // Check if we've seen this message before by looking at path
                        uint8_t path_nodes[3] = {0};
                        uint8_t path_length = 0;
                        decodeEncryptedPath(receivedMsg.encrypted_path, path_nodes);
                        
                        // Check if our node ID is already in the path (to avoid loops)
                        bool alreadyProcessed = false;
                        for (int i = 0; i < path_length; i++) {
                            if (path_nodes[i] == 0) {  // 0 is our node ID
                                alreadyProcessed = true;
                                break;
                            }
                        }
                        
                        both.printf("RX: Type=%d, Node=%d, Seq=%d (%d bytes) ", 
                                  receivedMsg.type, nodeId, sequence, rxLength);
                        
                        if (!alreadyProcessed && path_length < 3) {
                            // Determine priority based on message type
                            uint8_t priority = getMessagePriority(receivedMsg.type);
                            
                            // Create priority message for queue
                            PriorityMessage priorityMsg;
                            priorityMsg.message = receivedMsg;
                            priorityMsg.priority = priority;
                            
                            both.printf("Queuing (p=%d)\n", priority);
                            
                            // Send to priority queue
                            xQueueSend(priorityQueue, &priorityMsg, portMAX_DELAY);
                        } else {
                            both.println("Skipped (already in path)");
                        }
                    } else {
                        both.println("Failed to decode as protobuf");
                    }
                }
                
                // Restart receiver
                RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
            }
            
            xSemaphoreGive(xSemaphore);  // Release the semaphore
        }
        
        vTaskDelay(RX_TASK_DELAY / portTICK_PERIOD_MS);  // Small delay
    }
}

// Setup Function
void setup() {
    heltec_setup();
    both.println("\n\n\nProto Repeater\n");
    delay(1500);
    both.println("LoRa Radio Init");

    // Initialize LoRa module
    RADIOLIB_OR_HALT(radio.begin());
    radio.setDio1Action(onReceiveInterrupt);

    delay(1000);
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

    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

    // Initialize Semaphore
    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL) {
        both.println("Failed to create semaphore!");
        while (true);
    }
    xSemaphoreGive(xSemaphore);  // Initially free

    // Initialize priority queue
    // The queue will sort by priority (uint8_t in PriorityMessage struct)
    priorityQueue = xQueueCreate(10, sizeof(PriorityMessage));

    if (priorityQueue == NULL) {
        both.println("Error: Priority queue creation failed!");
        while (true);
    }

    // Create RTOS Tasks
    xTaskCreate(sendMessage, "Send Task", 4096, NULL, 1, &sendTaskHandle);
    xTaskCreate(receiveMessages, "Receive Task", 4096, NULL, 1, &receiveTaskHandle);

    both.println("LoRa Proto Repeater Initialized");
}

// Loop Function
void loop() {
    heltec_loop();  // Keep system running

    if(button.isDoubleClick()) {
        if(display_is_on) {
            both.println("\n\n\n\nTurning Display Off\n\n");
            delay(800);
            display.displayOff();
            display_is_on = !display_is_on;
        } else {
            display.displayOn();
            display_is_on = !display_is_on;
        }
    }
}