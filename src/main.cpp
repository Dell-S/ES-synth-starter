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

// Output multiplexer bits
const int DEN_BIT  = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Global variable for the current phase step size (used in sound generation)
// This variable is declared volatile because it is accessed both from the main (or task) context
// and from the interrupt service routine.
volatile uint32_t currentStepSize = 0;

// Global structure for sharing system state (the key matrix inputs)
struct SystemState {
  std::bitset<32> inputs;
} sysState;

// Precomputed phase step sizes for the 12 musical keys (C, C♯, D, D♯, E, F, F♯, G, G♯, A, A♯, B)
// Equal temperament is used and tuning is based on A = 440Hz (which is key index 9).
// The step size S for a given frequency f is computed as: 
//     S = (2^32 * f) / 22000
const uint32_t stepSizes[12] = {
  51080511,  // C
  54113906,  // C♯
  57362353,  // D
  60709836,  // D♯
  64341089,  // E
  68195786,  // F
  72268860,  // F♯
  76537834,  // G
  81084437,  // G♯
  85899334,  // A (440Hz)
  90965186,  // A♯
  96468917   // B
};

// Hardware timer used for generating the sound sample rate.
HardwareTimer sampleTimer(TIM1);

// -----------------------------------------------------------------------------
// Function: setOutMuxBit()
// Description: Sets a multiplexer bit using the key matrix. (As provided.)
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Function: setRow()
// Description: Selects a given row in the key matrix.
//              The row select enable (REN) is first disabled, the address pins (RA0-RA2)
//              are set to the required value, then REN is re-enabled.
// -----------------------------------------------------------------------------
void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);  // Disable REN while changing address pins
  digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);
  digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
  digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
  digitalWrite(REN_PIN, HIGH); // Re-enable REN
}

// -----------------------------------------------------------------------------
// Function: readCols()
// Description: Reads the four columns (C0–C3) of the key matrix and returns a 
//              std::bitset<4> containing the state of each column.
// -----------------------------------------------------------------------------
std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = (digitalRead(C0_PIN) == HIGH);
  result[1] = (digitalRead(C1_PIN) == HIGH);
  result[2] = (digitalRead(C2_PIN) == HIGH);
  result[3] = (digitalRead(C3_PIN) == HIGH);
  return result;
}

// -----------------------------------------------------------------------------
// Function: sampleISR()
// Description: Interrupt Service Routine called at 22kHz to generate the sawtooth wave.
//              It updates a 32-bit phase accumulator and converts the phase to an 8-bit 
//              output (with a DC offset) which is then sent to the analogue output.
// -----------------------------------------------------------------------------
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  // Adjust the output so that 0 becomes the mid-point (DC offset) of the analogue output range
  analogWrite(OUTR_PIN, Vout + 128);
}

// -----------------------------------------------------------------------------
// Task: scanKeysTask()
// Description: Scans the key matrix by selecting each row, reading the columns,
//              and then storing the complete state (first 12 keys) in sysState.inputs.
//              It also determines which key (if any) is pressed and updates currentStepSize.
//              The task executes every 50ms.
// -----------------------------------------------------------------------------
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    uint32_t localStepSize = 0;
    // Loop through rows 0 to 2 (covering 12 keys)
    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3); // Allow time for the row to settle
      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        inputs[row * 4 + col] = cols[col];
        // For the first 12 keys (rows 0–2): if the key is pressed (active low),
        // use its corresponding phase step size.
        if (row * 4 + col < 12) {
          if (!cols[col]) { // Key pressed (logic 0)
            localStepSize = stepSizes[row * 4 + col];
          }
        }
      }
    }
    sysState.inputs = inputs;
    // Atomically update the global currentStepSize variable
    __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
  }
}

// -----------------------------------------------------------------------------
// Task: displayUpdateTask()
// Description: Updates the OLED display every 100ms. It prints a "Helllo World!" 
//              message and displays the state of the key matrix (as a hexadecimal number).
//              It also toggles the onboard LED.
// -----------------------------------------------------------------------------
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static uint32_t count = 0;
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

// -----------------------------------------------------------------------------
// setup()
// Description: Performs initialisation for pins, display, UART, the analogue output,
//              and the hardware timer. It also creates the FreeRTOS tasks and starts the scheduler.
// -----------------------------------------------------------------------------
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

  // Create the key scanning task (higher priority)
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,   /* Task function */
    "scanKeys",     /* Task name */
    64,             /* Stack size in words */
    NULL,           /* Parameter (unused) */
    2,              /* Priority */
    &scanKeysHandle /* Task handle */
  );

  // Create the display update task (lower priority)
  TaskHandle_t displayHandle = NULL;
  xTaskCreate(
    displayUpdateTask,  /* Task function */
    "displayUpdate",    /* Task name */
    256,                /* Stack size in words */
    NULL,               /* Parameter (unused) */
    1,                  /* Priority */
    &displayHandle      /* Task handle */
  );

  // Start the FreeRTOS scheduler. (This call does not return.)
  vTaskStartScheduler();
}

// -----------------------------------------------------------------------------
// loop()
// Description: Empty because FreeRTOS tasks are now running independently.
// -----------------------------------------------------------------------------
void loop() {
  // Nothing to do here.
}
