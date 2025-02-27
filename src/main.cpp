#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>
#include <ES_CAN.h>
#include <string.h>  // for memcpy()

//------------------------
// Module configuration:
// Uncomment one of the following:
// For a module that sends key messages but does not generate sound:
//// #define SENDER_MODE
// For a module that receives messages and generates sound from local keys and incoming messages:
#define RECEIVER_MODE
//------------------------
// Uncomment to disable threads (for execution time testing)
// #define DISABLE_THREADS

// Uncomment to test execution time of scanKeysTask (the test version runs 32 iterations)
 // #define TEST_SCANKEYS

// Pin definitions
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN  = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;

const int OUT_PIN = D11;
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

const int DEN_BIT  = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Global structure for system state with mutex, knob rotation, and current octave.
struct SystemState {
  std::bitset<32> inputs;
  int knob3Rotation;      // Knob 3 rotation (volume control, limited 0-8)
  uint8_t currentOctave;   // Current octave (0 to 8)
  SemaphoreHandle_t mutex;
} sysState;

// Precomputed phase step sizes for the 12 musical keys (C, C#, D, D#, E, F, F#, G, G#, A, A#, B)
const uint32_t stepSizes[12] = {
  51080511,  // C
  54113906,  // C#
  57362353,  // D
  60709836,  // D#
  64341089,  // E
  68195786,  // F
  72268860,  // F#
  76537834,  // G
  81084437,  // G#
  85899334,  // A (440Hz)
  90965186,  // A#
  96468917   // B
};

// Hardware timer used for generating sound at a sample rate of 22kHz.
HardwareTimer sampleTimer(TIM1);

// Global variables used for sound generation and note display
volatile uint32_t currentStepSize = 0;
volatile int currentNoteIndex = -1;  // -1 indicates no key is pressed

// CAN bus globals
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
volatile uint8_t RX_Message[8] = {0};  // Global variable to hold the received CAN message

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//------------------------------------------------
// Utility functions for the key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);  // Disable REN while updating address pins
  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
  digitalWrite(REN_PIN, HIGH); // Re-enable REN
}

std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = (digitalRead(C0_PIN) == HIGH);
  result[1] = (digitalRead(C1_PIN) == HIGH);
  result[2] = (digitalRead(C2_PIN) == HIGH);
  result[3] = (digitalRead(C3_PIN) == HIGH);
  return result;
}

//------------------------------------------------
// ISR for sound generation (22kHz sample rate)
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  // Volume control via knob3Rotation (log taper)
  int knob = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
  Vout = Vout >> (8 - knob);
  analogWrite(OUTR_PIN, Vout + 128);
}

//------------------------------------------------
// scanKeysTask: scans key matrix, decodes knob3, processes joystick for octave changes,
// and (in sender mode) sends CAN messages. Runs every 20ms.
#ifndef TEST_SCANKEYS
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // For knob3 decode: store previous {B,A} state (2 bits)
  static uint8_t prevKnobState = 0;
  // For CAN message generation: keep a copy of previous state for keys 0-11.
  static std::bitset<32> prevInputs;
  
  // For joystick octave change – used to ensure only one change per extreme push.
  static bool octaveChanged = false;
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    uint32_t localStepSize = 0;
    int localNoteIndex = -1; // -1 indicates no key pressed

    // Loop through rows 0 to 3 (rows 0-2: note keys; row 3: knob3)
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3); // Allow time for the row to settle
      std::bitset<4> cols = readCols();
      
      if (row < 3) {
        // Process keys for note generation (only first 12 keys)
        for (uint8_t col = 0; col < 4; col++) {
          inputs[row * 4 + col] = cols[col];
          if ((row * 4 + col) < 12) {
            if (!cols[col]) { // key pressed (active low)
              localStepSize = stepSizes[row * 4 + col];
              localNoteIndex = row * 4 + col;
            }
          }
        }
      } else {
        // Row 3: decode knob3 (using columns 0 and 1 for signals A and B)
        uint8_t currKnobState = ((cols[1] ? 1 : 0) << 1) | (cols[0] ? 1 : 0);
        int delta = 0;
        // Decode state transitions
        if (prevKnobState == 0x0 && currKnobState == 0x1) {
          delta = 1;
        } else if (prevKnobState == 0x1 && currKnobState == 0x0) {
          delta = -1;
        } else if (prevKnobState == 0x2 && currKnobState == 0x3) {
          delta = -1;
        } else if (prevKnobState == 0x3 && currKnobState == 0x2) {
          delta = 1;
        }
        // Update knob3Rotation with limits 0 to 8 (protected by mutex)
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.knob3Rotation += delta;
        if (sysState.knob3Rotation > 8) sysState.knob3Rotation = 8;
        if (sysState.knob3Rotation < 0) sysState.knob3Rotation = 0;
        xSemaphoreGive(sysState.mutex);
        prevKnobState = currKnobState;
        // Optionally store knob inputs in bits 12 and 13 (for debugging)
        inputs[12] = cols[0];
        inputs[13] = cols[1];
      }
    }
    
    // Update the global key state with mutex protection.
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = inputs;
    xSemaphoreGive(sysState.mutex);
    
    // Update the sound generation variables (only in RECEIVER_MODE)
    #ifdef RECEIVER_MODE
      if (localNoteIndex >= 0) {
        uint8_t octave;
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        octave = sysState.currentOctave;
        xSemaphoreGive(sysState.mutex);
        uint32_t newStepSize;
        if (octave >= 4)
          newStepSize = localStepSize << (octave - 4);
        else
          newStepSize = localStepSize >> (4 - octave);
        __atomic_store_n(&currentStepSize, newStepSize, __ATOMIC_RELAXED);
        __atomic_store_n(&currentNoteIndex, localNoteIndex, __ATOMIC_RELAXED);
      } else {
        __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&currentNoteIndex, localNoteIndex, __ATOMIC_RELAXED);
      }
    #else
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
      __atomic_store_n(&currentNoteIndex, localNoteIndex, __ATOMIC_RELAXED);
    #endif


    // In SENDER_MODE, compare current keys with previous state and generate a CAN message for each change.
    #ifdef SENDER_MODE
      for (int key = 0; key < 12; key++) {
        bool currentPressed = !inputs.test(key); // active low: false means pressed
        bool prevPressed = !prevInputs.test(key);
        if (currentPressed != prevPressed) {
          uint8_t TX_Message[8] = {0};
          TX_Message[0] = currentPressed ? 'P' : 'R';
          uint8_t currentOctave;
          xSemaphoreTake(sysState.mutex, portMAX_DELAY);
          currentOctave = sysState.currentOctave;
          xSemaphoreGive(sysState.mutex);
          TX_Message[1] = currentOctave;    // use current octave
          TX_Message[2] = key;              // note number
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        }
      }
      prevInputs = inputs;
    #endif

    // --- Joystick-based Octave Change ---
    // Read the analog value from the joystick Y-axis.
    int joyVal = analogRead(JOYY_PIN);
    // Use thresholds (example: <400 to increment, >600 to decrement)
    if (!octaveChanged) {
      if (joyVal < 400) {
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        if (sysState.currentOctave < 8) sysState.currentOctave++;
        xSemaphoreGive(sysState.mutex);
        octaveChanged = true;
      } else if (joyVal > 600) {
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        if (sysState.currentOctave > 0) sysState.currentOctave--;
        xSemaphoreGive(sysState.mutex);
        octaveChanged = true;
      }
    } else {
      // Only allow another change once the joystick returns to center.
      if (joyVal >= 400 && joyVal <= 600) {
        octaveChanged = false;
      }
    }
  }
}
#else
// Test version of scanKeysTask: runs one iteration and generates a key press for each of the 12 keys.
void scanKeysTaskTest() {
  for (int key = 0; key < 12; key++) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = 'P';
    TX_Message[1] = 4; // Not used in test
    TX_Message[2] = key;
    #ifdef SENDER_MODE
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    #endif
  }
}
#endif

//------------------------------------------------
// displayUpdateTask: updates the OLED display with an aesthetic layout.
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    
    // Get shared state with mutex protection.
    int knobVal;
    unsigned long inputVal;
    uint8_t octave;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    knobVal = sysState.knob3Rotation;
    inputVal = sysState.inputs.to_ulong();
    octave = sysState.currentOctave;
    xSemaphoreGive(sysState.mutex);
    
    // ----- Title -----
    u8g2.setFont(u8g2_font_helvB08_tr);
    const char* title = "Synthesizer";
    int titleWidth = u8g2.getStrWidth(title);
    u8g2.drawStr((128 - titleWidth) / 2, 8, title);
    
    // ----- Volume Bar -----
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 16, "Vol:");
    // Draw volume bar: a framed rectangle with fill proportional to knobVal (0–8).
    const int barX = 40;
    const int barY = 10;
    const int barW = 70;
    const int barH = 6;
    u8g2.drawFrame(barX, barY, barW, barH);
    int fillWidth = (knobVal * barW) / 8;
    u8g2.drawBox(barX, barY, fillWidth, barH);
    
    // ----- Octave and Key Matrix -----
    char octStr[10];
    sprintf(octStr, "Oct:%d", octave);
    u8g2.drawStr(2, 28, octStr);
    
    char hexBuffer[9];
    sprintf(hexBuffer, "%08lX", inputVal);
    u8g2.drawStr(60, 28, hexBuffer);
    
    // ----- Note or TX Indicator -----
    #ifdef RECEIVER_MODE
      const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
      int noteIndex = __atomic_load_n(&currentNoteIndex, __ATOMIC_RELAXED);
      if (noteIndex >= 0 && noteIndex < 12) {
        u8g2.drawStr(60, 16, noteNames[noteIndex]);
      }
    #else
      u8g2.drawStr(60, 16, "TX");
    #endif
    
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

//------------------------------------------------
// decodeTask: processes received CAN messages (only in RECEIVER_MODE)
#ifdef RECEIVER_MODE
void decodeTask(void * pvParameters) {
  uint8_t localMsg[8];
  for (;;) {
    xQueueReceive(msgInQ, localMsg, portMAX_DELAY);
    // Copy the message to the global RX_Message with mutex protection.
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy((void*)RX_Message, localMsg, 8);
    xSemaphoreGive(sysState.mutex);
    
    // Process the message: play note on key press, stop on release.
    if (localMsg[0] == 'R') {
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
    } else if (localMsg[0] == 'P') {
      uint8_t note = localMsg[2];
      uint8_t octave = localMsg[1];
      uint32_t step = stepSizes[note];
      uint32_t newStep;
      if (octave >= 4)
        newStep = step << (octave - 4);
      else
        newStep = step >> (4 - octave);
      __atomic_store_n(&currentStepSize, newStep, __ATOMIC_RELAXED);
    }
  }
}
#endif

//------------------------------------------------
// CAN_TX_Task: sends outgoing CAN messages (only in SENDER_MODE)
#ifdef SENDER_MODE
void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}
#endif

//------------------------------------------------
// CAN RX ISR: called when a CAN message is received.
void CAN_RX_ISR(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// CAN TX ISR: called when a mailbox becomes available (only in SENDER_MODE)
#ifdef SENDER_MODE
void CAN_TX_ISR(void) {
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
#endif

//------------------------------------------------
// setup() function
void setup() {
  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Initialise hardware timer for sound generation at 22kHz (only in RECEIVER_MODE)
  #ifdef RECEIVER_MODE
    sampleTimer.setOverflow(22000, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();
  #endif

  // Create the mutex for sysState and initialise knob3Rotation and currentOctave.
  sysState.mutex = xSemaphoreCreateMutex();
  sysState.knob3Rotation = 8;   // start with full volume
  sysState.currentOctave = 4;     // default octave

  // Initialise CAN bus.
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  msgInQ = xQueueCreate(36, 8);
  #ifdef SENDER_MODE
    msgOutQ = xQueueCreate(36, 8);
    CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
    CAN_RegisterISR_TX(CAN_TX_ISR);
  #endif
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_Start();

  #ifndef DISABLE_THREADS
    // Create tasks
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask,    /* Task function */
      "scanKeys",      /* Task name */
      128,             /* Stack size in words */
      NULL,            /* Parameter */
      2,               /* Priority */
      &scanKeysHandle  /* Task handle */
    );
    
    TaskHandle_t displayHandle = NULL;
    xTaskCreate(
      displayUpdateTask,
      "displayUpdate",
      256,
      NULL,
      1,
      &displayHandle
    );
    
    #ifdef RECEIVER_MODE
      TaskHandle_t decodeHandle = NULL;
      xTaskCreate(
        decodeTask,
        "decode",
        128,
        NULL,
        2,
        &decodeHandle
      );
    #endif
    
    #ifdef SENDER_MODE
      TaskHandle_t canTxHandle = NULL;
      xTaskCreate(
        CAN_TX_Task,
        "canTx",
        128,
        NULL,
        3,
        &canTxHandle
      );
    #endif
    
    // Start the FreeRTOS scheduler (this call does not return)
    vTaskStartScheduler();
  #endif

  // Test execution time for scanKeysTask if enabled.
  #ifdef TEST_SCANKEYS
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++) {
      scanKeysTaskTest();
    }
    Serial.println(micros()-startTime);
    while(1);
  #endif
}

void loop() {
  // Nothing to do here. The RTOS scheduler now manages all tasks.
}
