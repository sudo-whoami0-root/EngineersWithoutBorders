// ============================
// Heltec LoRa Ping-Pong + Sensor Burst
// Behavior:
// - Take ONE new sensor reading at the start of each burst.
// - Build ONE payload string from that reading.
// - Send the SAME payload 5 times spaced across BURST_PERIOD_MS.
// - After EACH TX, go into RX waiting for an incoming packet.
// - If an "ACK" is received, immediately restart with a NEW reading/burst.
// Fix applied:
// - Moving average no longer "ramps up" due to initial zeros.
//   Uses sampleCount to divide by the number of valid samples collected so far.
// ============================

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "stdlib.h"

// ---------------- LoRa config ----------------
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

// Wait for a response after each send (ms)
#define RX_TIMEOUT_VALUE                            1200

#define BUFFER_SIZE                                 64

// ---------------- Burst behavior ----------------
static const uint8_t  BURST_COUNT      = 5;
static const uint32_t BURST_PERIOD_MS  = 100000;              // every 100 seconds
static const uint32_t INTER_SEND_MS    = BURST_PERIOD_MS / 5; // 20000ms

// ---------------- Sensor config ----------------
#define ANALOG_PIN                                  1  // <-- set your actual ADC pin

// Moving average settings
static const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;
float total_height = 0.0f;
float average_height = 0.0f;

// NEW: how many valid samples we have so far
uint8_t sampleCount = 0;

// Your calibration/offset params
float trimPressure = 0.43;
float zeroHeight   = 2.80;

float pressure_kpa = 0.0f;
float cm_h2o       = 0.0f;

// ---------------- Radio globals ----------------
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
char burstPacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);

// ---------------- State machine ----------------
typedef enum {
  LOWPOWER,
  STATE_SCHEDULE,
  STATE_RX
} States_t;

States_t state;

int16_t Rssi = 0;
int16_t rxSize = 0;
int16_t txNumber = 0;

// Scheduler variables
uint32_t burstStartMs = 0;
uint8_t  sendIndex = 0;

// ACK-triggered restart
volatile bool ackRestartRequested = false;
#define ACK_TOKEN "ACK"

// ---------------- Sensor update ----------------
// calculate output data ONCE per burst
static void updateSensorAverageOnce() {
  // int raw = analogRead(ANALOG_PIN);

  // // Clamp just in case
  // if (raw < 0) raw = 0;
  // if (raw > 4095) raw = 4095;

  // // ESP32 12-bit ADC typical range 0..4095
  // float voltage = raw * (3.3f / 4095.0f);

  // Serial.println(voltage);
  // // Your existing formula
  // // If your pressure sensor is powered by 3.3V,
  // // consider changing /5.0 -> /3.3
  // //pressure_kpa = (55.55f * voltage / 5.0f) - 2.22f + trimPressure;

  // pressure_kpa = 11.11f * voltage - 2.22f;
  // // kPa -> cm H2O
  // cm_h2o = 0.0101972f * pressure_kpa * 1000.0f;

  // // Moving average ring buffer update
  // // Remove old sample at this slot
  // total_height -= readings[readIndex];
  // // Store newest
  // readings[readIndex] = cm_h2o;
  // // Add newest
  // total_height += readings[readIndex];

  // // Advance index
  // readIndex++;
  // if (readIndex >= numReadings) readIndex = 0;

  // // NEW: grow sampleCount until buffer is full
  // if (sampleCount < numReadings) sampleCount++;

  // // Compute average using only valid samples so far
  // average_height = (total_height / sampleCount) - zeroHeight;

    average_height = random(0, 1000) / 100.0; // e.g. 0.00–9.99
    // average_height = 11.11;
  
}

// ---------------- Burst control ----------------
static void startNewBurst() {
  burstStartMs = millis();
  sendIndex = 0;

  updateSensorAverageOnce();

  String dataString = String(average_height, 2);

  txNumber++;

  snprintf(burstPacket, BUFFER_SIZE, "%s", dataString.c_str());

  Serial.printf("\n--- New burst #%d ---\n", txNumber);
  Serial.printf("Average height: %.2f cm H2O\n", average_height);
  Serial.printf("Burst payload: \"%s\"\n", burstPacket);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Init readings to 0
  for (int i = 0; i < numReadings; i++) readings[i] = 0.0f;

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  state = STATE_SCHEDULE;
}

// ---------------- Loop ----------------
void loop() {
  switch (state) {

    case STATE_SCHEDULE: {
      uint32_t now = millis();

      // If ACK asked us to restart, start a new burst immediately
      if (ackRestartRequested) {
        ackRestartRequested = false;
        startNewBurst();
        now = millis();
      }

      // Start first burst or roll every BURST_PERIOD_MS
      if (burstStartMs == 0 || (uint32_t)(now - burstStartMs) >= BURST_PERIOD_MS) {
        startNewBurst();
      }

      // Send up to 5 repeats on schedule
      if (sendIndex < BURST_COUNT) {
        uint32_t nextSendTime = burstStartMs + (sendIndex * INTER_SEND_MS);

        if ((int32_t)(now - nextSendTime) >= 0) {
          snprintf(txpacket, BUFFER_SIZE, "%s", burstPacket);

          Serial.printf("\n-----------------------------------------\n");
          Serial.printf("Attempt %u/%u to send\n", sendIndex + 1, BURST_COUNT);
          Serial.printf("sending packet \"%s\" , length %d\n",
                        txpacket, (int)strlen(txpacket));

          Radio.Send((uint8_t *)txpacket, strlen(txpacket));
          sendIndex++;

          state = LOWPOWER;  // wait for TX done IRQ
        } else {
          state = LOWPOWER;
        }
      } else {
        state = LOWPOWER;
      }
      break;
    }

    case STATE_RX:
      Serial.println("into RX mode");
      Radio.Rx(RX_TIMEOUT_VALUE);
      state = LOWPOWER;
      break;

    case LOWPOWER:
      Radio.IrqProcess();

      // If no IRQ moved us elsewhere, re-enter schedule
      if (state == LOWPOWER) {
        state = STATE_SCHEDULE;
      }
      break;

    default:
      state = STATE_SCHEDULE;
      break;
  }
}

// ---------------- Callbacks ----------------
void OnTxDone(void) {
  Serial.print("TX done......\n");
  state = STATE_RX;   // after every send, go RX
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.print("TX Timeout......\n");
  state = STATE_SCHEDULE;
}

void OnRxTimeout(void) {
  Radio.Sleep();
  Serial.println("RX Timeout......");
  state = STATE_SCHEDULE;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Rssi = rssi;
  rxSize = size;

  if (size >= BUFFER_SIZE) size = BUFFER_SIZE - 1;

  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';

  Radio.Sleep();

  Serial.printf("\nreceived packet \"%s\" with Rssi %d , length %d , SNR %d\n",
                rxpacket, Rssi, rxSize, snr);

  // If ACK received, restart with a NEW reading/burst
  if (strcmp(rxpacket, ACK_TOKEN) == 0) {
    Serial.println("ACK received -> restarting with a new reading");

    ackRestartRequested = true;

    // Clear current burst state so next schedule pass starts immediately
    burstStartMs = 0;
    sendIndex = 0;
  } else {
    Serial.println("back to scheduler");
  }

  state = STATE_SCHEDULE;
}
