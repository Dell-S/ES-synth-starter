#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>
#include <ES_CAN.h>
#include <string.h>  // for memcpy() and string functions

//------------------------
// Increase volume resolution by defining MAX_VOLUME.
#define MAX_VOLUME 16

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

// The original joystick pins are still defined but not used for waveform selection now.
const int JOYY_PIN = A0; // used for octave change
const int JOYX_PIN = A1; // not used

const int DEN_BIT  = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

//------------------------
// Waveform selection support
enum WaveformType {
  SAWTOOTH = 0,
  SQUARE,
  TRIANGLE,
  SINE
};
volatile uint8_t waveformType = SAWTOOTH; // Default waveform

// Global structure for system state with mutex, knob rotation, and current octave.
struct SystemState {
  std::bitset<32> inputs;
  int knob3Rotation;      // Knob 3 rotation (volume control, 0 to MAX_VOLUME)
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

//----------------------------------------------------------------
// Polyphony support: define maximum voices and a Voice structure.
#define MAX_VOICES 4

struct Voice {
  bool active;
  uint32_t phaseAcc;
  uint32_t stepSize;
  uint8_t key; // note number (0-11)
};

volatile Voice voices[MAX_VOICES] = {0};  // Global array of voices

//----------------------------------------------------------------
// Sine lookup table (256 values) computed as 128+127*sin(2*pi*i/256)
static const uint8_t sineTable[256] = {
  128,131,134,137,140,143,146,149,152,155,158,161,163,166,169,171,
  174,177,179,182,184,186,189,191,193,195,197,199,201,203,205,206,
  208,210,211,213,214,215,217,218,219,220,221,222,223,224,224,225,
  226,226,227,227,228,228,228,229,229,229,229,230,230,230,230,230,
  230,230,230,230,230,230,229,229,229,229,228,228,228,227,227,226,
  226,225,224,224,223,222,221,220,219,218,217,215,214,213,211,210,
  208,206,205,203,201,199,197,195,193,191,189,186,184,182,179,177,
  174,171,169,166,163,161,158,155,152,149,146,143,140,137,134,131,
  128,124,121,118,115,112,109,106,103,100, 97, 94, 91, 88, 86, 83,
   81, 78, 76, 74, 71, 69, 67, 65, 63, 61, 59, 57, 55, 53, 51, 50,
   48, 46, 45, 43, 42, 40, 39, 38, 36, 35, 34, 33, 32, 31, 30, 29,
   28, 27, 26, 25, 24,24, 23, 22, 22, 21, 21, 20, 20, 19, 19, 19,
   18, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 14, 13,
   13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10,  9,  9,  9,  8,  8,
    8,  7,  7,  7,  6,  6,  6,  5,  5,  5,  4,  4,  4,  3,  3,  3,
    2,  2,  2,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

//----------------------------------------------------------------
// Hardware timer used for generating sound at a sample rate of 22kHz.
HardwareTimer sampleTimer(TIM1);

// CAN bus globals
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
volatile uint8_t RX_Message[8] = {0};  // Holds received CAN message

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
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
  digitalWrite(REN_PIN, HIGH);
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
// Polyphony helper functions.
void allocateVoice(uint8_t key, uint32_t baseStep) {
  uint8_t octave;
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  octave = sysState.currentOctave;
  xSemaphoreGive(sysState.mutex);
  
  uint32_t newStep;
  if (octave >= 4)
    newStep = baseStep << (octave - 4);
  else
    newStep = baseStep >> (4 - octave);
  
  noInterrupts();
  for (int i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active) {
      voices[i].active = true;
      voices[i].phaseAcc = 0;
      voices[i].stepSize = newStep;
      voices[i].key = key;
      break;
    }
  }
  interrupts();
}

void freeVoice(uint8_t key) {
  noInterrupts();
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].active && voices[i].key == key) {
      voices[i].active = false;
      break;
    }
  }
  interrupts();
}

void allocateVoiceCAN(uint8_t note, uint8_t octave, uint32_t baseStep) {
  uint32_t newStep;
  if (octave >= 4)
    newStep = baseStep << (octave - 4);
  else
    newStep = baseStep >> (4 - octave);
  
  noInterrupts();
  for (int i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active) {
      voices[i].active = true;
      voices[i].phaseAcc = 0;
      voices[i].stepSize = newStep;
      voices[i].key = note;
      break;
    }
  }
  interrupts();
}

//------------------------------------------------
// ISR for sound generation (22kHz sample rate) with waveform selection.
// Here the volume scaling uses a float multiplier computed from the knob (0 to MAX_VOLUME).
void sampleISR() {
  int32_t mixedOutput = 0;
  int activeCount = 0;
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].active) {
      voices[i].phaseAcc += voices[i].stepSize;
      int32_t voiceOutput = 0;
      switch(waveformType) {
        case SAWTOOTH:
          voiceOutput = ((voices[i].phaseAcc >> 24) - 128);
          break;
        case SQUARE:
          voiceOutput = (voices[i].phaseAcc & 0x80000000UL) ? 127 : -128;
          break;
        case TRIANGLE:
          if (voices[i].phaseAcc < 0x80000000UL)
            voiceOutput = ((voices[i].phaseAcc >> 23) - 128);
          else
            voiceOutput = (127 - ((voices[i].phaseAcc - 0x80000000UL) >> 23));
          break;
        case SINE:
          voiceOutput = ((int)sineTable[voices[i].phaseAcc >> 24]) - 128;
          break;
        default:
          voiceOutput = ((voices[i].phaseAcc >> 24) - 128);
      }
      mixedOutput += voiceOutput;
      activeCount++;
    }
  }
  if (activeCount > 0)
    mixedOutput /= activeCount;
  
  int knob = __atomic_load_n(&sysState.knob3Rotation, __ATOMIC_RELAXED);
  // Map knob (0 to MAX_VOLUME) to a volume factor between 0.1 and 1.0.
  float volFactor = 0.1 + ((float)knob / (float)MAX_VOLUME) * 0.9;
  int finalOutput = (int)(mixedOutput * volFactor);
  
  analogWrite(OUTR_PIN, finalOutput + 128);
}

//------------------------------------------------
// scanKeysTask: scans key matrix, decodes knob3 (volume), decodes knob1 (waveform),
// processes joystick for octave changes, and sends CAN messages (if SENDER_MODE).
// Runs every 20ms.
#ifndef TEST_SCANKEYS
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  static uint8_t prevKnobState = 0;    // For knob3 (volume)
  static uint8_t prevKnob1State = 0;     // For knob1 (waveform)
  static std::bitset<32> prevInputs;
  static bool octaveChanged = false;
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    
    // Process rows 0 to 2 for note keys.
    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        uint8_t keyIndex = row * 4 + col;
        inputs[keyIndex] = cols[col];
      }
    }
    
    // Process row 3 for knob3 (volume) decoding.
    {
      setRow(3);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      uint8_t currKnobState = ((cols[1] ? 1 : 0) << 1) | (cols[0] ? 1 : 0);
      int delta = 0;
      if (prevKnobState == 0x0 && currKnobState == 0x1)
        delta = 1;
      else if (prevKnobState == 0x1 && currKnobState == 0x0)
        delta = -1;
      else if (prevKnobState == 0x2 && currKnobState == 0x3)
        delta = -1;
      else if (prevKnobState == 0x3 && currKnobState == 0x2)
        delta = 1;
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      sysState.knob3Rotation += delta;
      if (sysState.knob3Rotation > MAX_VOLUME) sysState.knob3Rotation = MAX_VOLUME;
      if (sysState.knob3Rotation < 0) sysState.knob3Rotation = 0;
      xSemaphoreGive(sysState.mutex);
      prevKnobState = currKnobState;
    }
    
    // Process row 4 for knob1 (waveform selection) decoding.
    {
      setRow(4);
      delayMicroseconds(3);
      std::bitset<4> knob1Cols = readCols();
      uint8_t currKnob1State = ((knob1Cols[1] ? 1 : 0) << 1) | (knob1Cols[0] ? 1 : 0);
      int delta1 = 0;
      if (prevKnob1State == 0x0 && currKnob1State == 0x1)
        delta1 = 1;
      else if (prevKnob1State == 0x1 && currKnob1State == 0x0)
        delta1 = -1;
      else if (prevKnob1State == 0x2 && currKnob1State == 0x3)
        delta1 = -1;
      else if (prevKnob1State == 0x3 && currKnob1State == 0x2)
        delta1 = 1;
      int newWave = waveformType + delta1;
      if(newWave < 0) newWave = 3;
      if(newWave > 3) newWave = 0;
      waveformType = newWave;
      prevKnob1State = currKnob1State;
    }
    
    // Update global key state.
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = inputs;
    xSemaphoreGive(sysState.mutex);
    
    // For keys 0-11, detect changes to allocate or free voices.
    for (int key = 0; key < 12; key++) {
      bool currentPressed = !inputs.test(key);  // active low: false means pressed
      bool prevPressed = !prevInputs.test(key);
      if (currentPressed != prevPressed) {
        if (currentPressed) {
          allocateVoice(key, stepSizes[key]);
          #ifdef SENDER_MODE
            uint8_t TX_Message[8] = {0};
            TX_Message[0] = 'P';
            uint8_t currentOctave;
            xSemaphoreTake(sysState.mutex, portMAX_DELAY);
            currentOctave = sysState.currentOctave;
            xSemaphoreGive(sysState.mutex);
            TX_Message[1] = currentOctave;
            TX_Message[2] = key;
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          #endif
        } else {
          freeVoice(key);
          #ifdef SENDER_MODE
            uint8_t TX_Message[8] = {0};
            TX_Message[0] = 'R';
            uint8_t currentOctave;
            xSemaphoreTake(sysState.mutex, portMAX_DELAY);
            currentOctave = sysState.currentOctave;
            xSemaphoreGive(sysState.mutex);
            TX_Message[1] = currentOctave;
            TX_Message[2] = key;
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          #endif
        }
      }
    }
    prevInputs = inputs;
    
    // --- Joystick-based Octave Change (vertical axis: JOYY) ---
    int joyVal = analogRead(JOYY_PIN);
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
      if (joyVal >= 400 && joyVal <= 600) {
        octaveChanged = false;
      }
    }
  }
}
#else
void scanKeysTaskTest() {
  for (int key = 0; key < 12; key++) {
    #ifdef SENDER_MODE
      uint8_t TX_Message[8] = {0};
      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      TX_Message[2] = key;
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    #endif
  }
}
#endif

//------------------------------------------------
// Helper functions for drawing icons on the display.
void drawSawIcon(int x, int y) {
  u8g2.drawLine(x, y + 8, x + 4, y);
  u8g2.drawLine(x + 4, y, x + 8, y + 8);
}

void drawSquareIcon(int x, int y) {
  u8g2.drawFrame(x, y, 8, 8);
  u8g2.drawBox(x, y, 4, 8);
}

void drawTriangleIcon(int x, int y) {
  u8g2.drawTriangle(x, y + 8, x + 4, y, x + 8, y + 8);
}

void drawSineIcon(int x, int y) {
  u8g2.drawLine(x, y + 4, x + 2, y + 2);
  u8g2.drawLine(x + 2, y + 2, x + 4, y + 4);
  u8g2.drawLine(x + 4, y + 4, x + 6, y + 6);
  u8g2.drawLine(x + 6, y + 6, x + 8, y + 4);
}

void drawSpeakerIcon(int x, int y) {
  u8g2.drawTriangle(x, y + 4, x + 4, y, x + 4, y + 8);
  u8g2.drawBox(x + 5, y + 2, 2, 4);
}

//------------------------------------------------
// displayUpdateTask: updated display layout including waveform info and icons.
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  const char* waveformNames[4] = {"Saw", "Square", "Tri", "Sine"};
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    
    int knobVal;
    uint8_t octave;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    knobVal = sysState.knob3Rotation;
    octave = sysState.currentOctave;
    xSemaphoreGive(sysState.mutex);
    
    // Line 1: Title with waveform info and icon.
    u8g2.setFont(u8g2_font_helvB08_tr);
    char titleBuffer[32];
    sprintf(titleBuffer, "Synth: %s", waveformNames[waveformType]);
    int titleWidth = u8g2.getStrWidth(titleBuffer);
    u8g2.drawStr((128 - titleWidth) / 2, 10, titleBuffer);
    
    int iconX = 110;
    int iconY = 2;
    switch (waveformType) {
      case SAWTOOTH:
        drawSawIcon(iconX, iconY);
        break;
      case SQUARE:
        drawSquareIcon(iconX, iconY);
        break;
      case TRIANGLE:
        drawTriangleIcon(iconX, iconY);
        break;
      case SINE:
        drawSineIcon(iconX, iconY);
        break;
      default:
        drawSawIcon(iconX, iconY);
    }
    
    // Line 2: Volume bar with speaker icon.
    drawSpeakerIcon(0, 12);
    const int barX = 20;
    const int barY = 16;
    const int barW = 80;
    const int barH = 4;
    u8g2.drawFrame(barX, barY, barW, barH);
    int fillWidth = (knobVal * barW) / MAX_VOLUME;
    u8g2.drawBox(barX, barY, fillWidth, barH);
    
    // Line 3: Active notes and octave info.
    char notesBuffer[32];
    memset(notesBuffer, 0, sizeof(notesBuffer));
    sprintf(notesBuffer, "Oct:%d ", octave);
    for (int i = 0; i < MAX_VOICES; i++) {
      if (voices[i].active) {
        strcat(notesBuffer, noteNames[voices[i].key]);
        strcat(notesBuffer, " ");
      }
    }
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 30, notesBuffer);
    
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
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy((void*)RX_Message, localMsg, 8);
    xSemaphoreGive(sysState.mutex);
    
    if (localMsg[0] == 'R') {
      freeVoice(localMsg[2]);
    } else if (localMsg[0] == 'P') {
      uint8_t note = localMsg[2];
      uint8_t octave = localMsg[1];
      allocateVoiceCAN(note, octave, stepSizes[note]);
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

// CAN TX ISR: (only in SENDER_MODE)
#ifdef SENDER_MODE
void CAN_TX_ISR(void) {
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
#endif

//------------------------------------------------
// setup() function
void setup() {
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
  pinMode(JOYY_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);

  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);

  Serial.begin(9600);
  Serial.println("Hello World");

  #ifdef RECEIVER_MODE
    sampleTimer.setOverflow(22000, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();
  #endif

  sysState.mutex = xSemaphoreCreateMutex();
  sysState.knob3Rotation = MAX_VOLUME;   // Set full volume to MAX_VOLUME
  sysState.currentOctave = 4;            // default octave

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
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask,
      "scanKeys",
      128,
      NULL,
      2,
      &scanKeysHandle
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
    
    vTaskStartScheduler();
  #endif

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
  // RTOS scheduler manages tasks.
}
