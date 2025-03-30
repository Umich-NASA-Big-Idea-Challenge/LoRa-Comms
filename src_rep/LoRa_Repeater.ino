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

// RTOS Handles
SemaphoreHandle_t xSemaphore;
TaskHandle_t sendTaskHandle;
TaskHandle_t receiveTaskHandle;

// Shared Variables
QueueHandle_t messageQueue;
volatile bool isReceiving = false;  // True when receiving
String rxdata;
volatile bool rxFlag = false;  // Flag to indicate message reception
bool display_is_on = true;

// Interrupt Service Routine (ISR) for receiving messages
void IRAM_ATTR onReceiveInterrupt() {
    rxFlag = true;  // Flag message received
    isReceiving = true;  // Mark channel as busy
}

// Function to check if the channel is free
bool isChannelFree() {
    return !isReceiving;
}

// **Task: Send Messages**
void sendMessage(void *pvParameters) {
    String message;
    
    while (true) {
        // Wait for a message in the queue (blocks indefinitely)
        if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdTRUE) {
            //both.println("Msg in Queue");
            message = "reP: " +message;
            // Wait until the channel is free before claiming the semaphore
            while (!isChannelFree()) {
                both.println("Channel busy, waiting...");
                vTaskDelay(LBT_BACKOFF_DELAY / portTICK_PERIOD_MS);
            }

            // Try to acquire the semaphore for transmission
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                int retries = 0;

                while (retries < MAX_RETRIES) {
                    if (isChannelFree()) {
                        both.printf("Msg: [%s] ", message.c_str());
                        radio.clearDio1Action();  // Clear any pending RX interrupt
                        heltec_led(50);  // Blink LED during TX
                        RADIOLIB(radio.transmit(message.c_str()));
                        heltec_led(0);  // Turn off LED

                        if (_radiolib_status == RADIOLIB_ERR_NONE) {
                            both.printf("Sent\n");
                        } else {
                            both.printf("TX failed (%i)\n", _radiolib_status);
                        }
                        break;  // Exit retry loop if successful
                    } else {
                        // Channel is busy, backoff and retry
                        both.println("Channel busy, retrying...");
                        vTaskDelay(LBT_BACKOFF_DELAY + random(0, 100) / portTICK_PERIOD_MS);
                        retries++;
                    }
                }
                radio.setDio1Action(onReceiveInterrupt);  // Re-enable RX interrupt
                RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF)); //start receiver
                
                xSemaphoreGive(xSemaphore);  // Release the semaphore
                vTaskDelay(TRANSMISSION_DELAY / portTICK_PERIOD_MS);  // Delay before next transmission
            }
        }
    }
}

// **Task: Receive Messages**
void receiveMessages(void *pvParameters) {
    while (true) {
        // Attempt to acquire the semaphore for receiving
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            if (rxFlag) {
                rxFlag = false;  // Reset flag
                radio.readData(rxdata);
                isReceiving = false;  // Mark channel as free

                if (_radiolib_status == RADIOLIB_ERR_NONE) {
                    both.printf("Msg: [%s]\n", rxdata.c_str());
                    //both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
                    //both.printf("  SNR: %.2f dB\n", radio.getSNR());
                    xQueueSend(messageQueue, rxdata.c_str(), portMAX_DELAY);
                }
                RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
            }
            xSemaphoreGive(xSemaphore);  // Release the semaphore
        }
        vTaskDelay(RX_TASK_DELAY / portTICK_PERIOD_MS);  // Small delay to avoid busy-looping
    }
}

// **Setup Function**
void setup() {
    heltec_setup();
    both.println("\n\n\nRepeater 1\n");
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

    // Initialize queue with a size of 10 messages
    messageQueue = xQueueCreate(10, sizeof(String));

    if (messageQueue == NULL) {
        both.println("Error: Message queue creation failed!");
        while (true);
    }

    // Create RTOS Tasks
    xTaskCreate(sendMessage, "Send Task", 4096, NULL, 1, &sendTaskHandle);
    xTaskCreate(receiveMessages, "Receive Task", 4096, NULL, 1, &receiveTaskHandle);

    both.println("LoRa Initialized.");

}



// **Loop Function (Empty since RTOS is handling tasks)**
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
