#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>

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

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Global variables used for sound generation and note display
volatile uint32_t currentStepSize = 0;
volatile int currentNoteIndex = -1;  // -1 indicates no note is pressed

// Global structure for system state (key matrix input)
struct SystemState {
  std::bitset<32> inputs;
} sysState;

// Precomputed phase step sizes for the 12 musical keys (C, C#, D, D#, E, F, F#, G, G#, A, A#, B)
// Equal temperament is used and tuning is based on A = 440Hz (which is key index 9).
// The step size S for a given frequency f is: S = (2^32 * f) / 22000
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

// Function to set a multiplexer bit using the key matrix.
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

// Function to select a given row in the key matrix.
void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);  // Disable REN while updating address pins
  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
  digitalWrite(REN_PIN, HIGH); // Re-enable REN
}

// Function to read the four columns of the key matrix.
std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = (digitalRead(C0_PIN) == HIGH);
  result[1] = (digitalRead(C1_PIN) == HIGH);
  result[2] = (digitalRead(C2_PIN) == HIGH);
  result[3] = (digitalRead(C3_PIN) == HIGH);
  return result;
}

// Interrupt Service Routine (ISR) for sound generation.
// This function is triggered at 22kHz to update the phase accumulator and output the sawtooth waveform.
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

// Task: scanKeysTask()
// This task scans the key matrix, updates the global key state, and selects a note to play.
// It runs every 50ms.
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    uint32_t localStepSize = 0;
    int localNoteIndex = -1; // -1 indicates no key pressed

    // Loop through rows 0 to 2 (covering 12 keys)
    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3); // Allow time for the row to settle
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        inputs[row * 4 + col] = cols[col];
        // For the first 12 keys, if the key is pressed (logic 0), update step size and note index.
        if ((row * 4 + col) < 12) {
          if (!cols[col]) { // key pressed (active low)
            localStepSize = stepSizes[row * 4 + col];
            localNoteIndex = row * 4 + col;
          }
        }
      }
    }
    sysState.inputs = inputs;
    __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
    __atomic_store_n(&currentNoteIndex, localNoteIndex, __ATOMIC_RELAXED);
  }
}

// Task: displayUpdateTask()
// This task updates the OLED display every 100ms.
// It shows the hexadecimal key matrix state and the note name (if a key is pressed).
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint32_t count = 0;
  // Mapping of note indices to note names.
  const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);

    // Load the current note index and display the note if valid.
    int noteIndex = __atomic_load_n(&currentNoteIndex, __ATOMIC_RELAXED);
    if (noteIndex >= 0 && noteIndex < 12) {
      u8g2.setCursor(2, 30);
      u8g2.print(noteNames[noteIndex]);
    }
    
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

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
  setOutMuxBit(DRST_BIT, LOW);  // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Initialise hardware timer for sound generation at 22kHz sample rate
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create the key scanning task (priority 2)
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,    /* Task function */
    "scanKeys",      /* Task name */
    64,              /* Stack size in words */
    NULL,            /* Parameter */
    2,               /* Priority */
    &scanKeysHandle  /* Task handle */
  );

  // Create the display update task (priority 1)
  TaskHandle_t displayHandle = NULL;
  xTaskCreate(
    displayUpdateTask,  /* Task function */
    "displayUpdate",    /* Task name */
    256,                /* Stack size in words */
    NULL,               /* Parameter */
    1,                  /* Priority */
    &displayHandle      /* Task handle */
  );

  // Start the FreeRTOS scheduler (this call does not return)
  vTaskStartScheduler();
}

void loop() {
  // Nothing to do here. The RTOS scheduler now manages all tasks.
}
