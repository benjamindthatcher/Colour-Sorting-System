#include <RunningAverage.h>
#include <Wire.h>
#include <Servo.h>
#include <U8x8lib.h>

// OLED display using the lower-memory U8x8 library.
// This is used instead of the Adafruit SSD1306 library to reduce memory usage.
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);

// Colour sensor pins.
// S0 and S1 control frequency scaling.
// S2 and S3 select which colour filter is being read.
// SENSOR_OUT outputs the pulse signal from the colour sensor.
#define SENSOR_S0 A1
#define SENSOR_S1 A2
#define SENSOR_S2 A3
#define SENSOR_S3 9
#define SENSOR_OUT 10

// User controls.
// BTN_MAIN is the easiest-to-reach top board button and is used for select/confirm.
// BTN_BACK returns to the menu.
// BTN_REJECT manually rejects/ejects a token if it becomes stuck or needs overriding.
#define BTN_MAIN   4
#define BTN_BACK   3
#define BTN_REJECT 2
#define POT_PIN A0

// Output devices.
// The servo physically sorts the token.
// The buzzer gives audio feedback.
// The relay provides clicks for UI feedback and completion signals.
// The LEDs show system state: ready, busy, or done.
#define SERVO_PIN 11
#define BUZZER_PIN 12
#define RELAY_PIN 13

#define LED_READY 6
#define LED_BUSY 7
#define LED_DONE 8

// Servo positions.
// 90 degrees is the sensor/resting position.
// 0 degrees ejects to the left cup.
// 180 degrees ejects to the right cup.
#define LEFT_CUP 0
#define SENSOR_POS 90
#define RIGHT_CUP 180

// Sensor smoothing.
// NUM_SAMPLES controls how many readings are averaged each time a colour is read.
// Higher values are more stable but slower.
#define NUM_SAMPLES 10

// Colour table sizes.
// NUM_COLOURS includes the 8 token colours plus EMPTY.
// NUM_TOKEN_COLOURS is only the real token colours, excluding EMPTY.
#define NUM_COLOURS 9
#define NUM_TOKEN_COLOURS 8

// Number of items in the main menu.
#define MENU_ITEMS 3

// Timing settings.
// TOKEN_CONFIRM_DELAY gives the token time to settle after it is first detected.
// SETTLE_DELAY gives the final token position time to stabilise before classification.
// SORT_DELAY holds the servo over the cup long enough for the token to drop.
// SERVO_STEP_DELAY controls how smooth/fast the servo movement is.
#define TOKEN_CONFIRM_DELAY 500
#define SETTLE_DELAY 2000
#define SORT_DELAY 800
#define SERVO_STEP_DELAY 3

// Number of tokens expected in one full run.
// Once this many tokens have been sorted, the run-complete screen and relay signal happen.
#define TOKENS_PER_RUN 8

// RunningAverage objects store several readings and return their average.
// This reduces random sensor noise and improves colour classification.
RunningAverage redAverage(NUM_SAMPLES);
RunningAverage greenAverage(NUM_SAMPLES);
RunningAverage blueAverage(NUM_SAMPLES);

// Servo object used to control the sorting arm.
Servo sorterServo;

// Counters displayed on the OLED during sorting.
int acceptedCount = 0;
int rejectedCount = 0;

// Counts how many tokens have been sorted in the current run only.
// This resets when a new run starts.
int sortedThisRun = 0;

// Menu and selected accepted colour indexes.
// menuIndex is controlled by the potentiometer in the main menu.
// acceptedColourIndex is controlled by the potentiometer in colour selection mode.
int menuIndex = 0;
int acceptedColourIndex = 0;

// Main program modes.
// START_SCREEN shows controls at power-on.
// MAIN_MENU lets the user choose mode.
// RUN_SORTER performs automatic sorting.
// SELECT_COLOUR chooses which token colour should be accepted.
// CALIBRATION_MODE records new colour readings.
enum AppMode {
  START_SCREEN,
  MAIN_MENU,
  RUN_SORTER,
  SELECT_COLOUR,
  CALIBRATION_MODE
};

// Sorting state machine.
// These states make the sorter behave in a controlled sequence instead of doing everything at once.
enum SorterState {
  WAITING_FOR_TOKEN,
  CONFIRMING_TOKEN,
  READING_TOKEN,
  SORTING_TOKEN,
  WAITING_FOR_REMOVAL
};

AppMode appMode = START_SCREEN;
SorterState sorterState = WAITING_FOR_TOKEN;

// Stores the name and calibrated RGB pulse-width values for each colour.
struct ColourCalibration {
  const char* name;
  int r;
  int g;
  int b;
};

// Calibrated colour values.
// EMPTY represents the sensor looking at the empty housing/no token.
ColourCalibration colours[NUM_COLOURS] = {
  {"RED", 1030, 1779, 1209},
  {"GREEN", 1796, 1680, 1247},
  {"BLUE", 1556, 1516, 833},
  {"BLACK", 1739, 1934, 1336},
  {"WHITE", 585, 533, 361},
  {"ORANGE", 455, 988, 812},
  {"YELLOW", 587, 632, 755},
  {"PURPLE", 1400, 1548, 866},
  {"EMPTY", 1595, 1632, 1127}
};

void setup() {
  // Configure colour sensor pins.
  pinMode(SENSOR_S0, OUTPUT);
  pinMode(SENSOR_S1, OUTPUT);
  pinMode(SENSOR_S2, OUTPUT);
  pinMode(SENSOR_S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);

  // Configure button inputs.
  // These buttons are active HIGH on this PCB, so no INPUT_PULLUP is used.
  pinMode(BTN_MAIN, INPUT);
  pinMode(BTN_BACK, INPUT);
  pinMode(BTN_REJECT, INPUT);

  // Configure outputs.
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  pinMode(LED_READY, OUTPUT);
  pinMode(LED_BUSY, OUTPUT);
  pinMode(LED_DONE, OUTPUT);

  // Ensure relay starts off.
  digitalWrite(RELAY_PIN, LOW);

  // Set startup LED state.
  setStatusReady();

  // Set the TCS3200/TCS230 colour sensor to 20% frequency scaling.
  // This makes the output frequency easier for the Arduino to read reliably.
  digitalWrite(SENSOR_S0, HIGH);
  digitalWrite(SENSOR_S1, LOW);

  // Start Serial Monitor for debugging and calibration printout.
  Serial.begin(115200);

  // Start OLED display.
  display.begin();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.clear();

  // Attach servo and move it to the central sensor position.
  sorterServo.attach(SERVO_PIN);
  sorterServo.write(SENSOR_POS);

  // Show startup controls on OLED and Serial Monitor.
  showStart();
  printStartup();
}

void loop() {
  // Manual reject is checked first so it works from almost anywhere.
  // This can be used to clear a stuck token by forcing the servo left.
  if (buttonPressed(BTN_REJECT)) {
    manualReject();
    return;
  }

  // On first power-up, the system stays on the control screen until S1 is pressed.
  if (appMode == START_SCREEN) {
    if (buttonPressed(BTN_MAIN)) {
      relayClick();
      appMode = MAIN_MENU;
      showMenu();
    }
    return;
  }

  // Back button returns to the main menu.
  if (buttonPressed(BTN_BACK)) {
    relayClick();
    goHome();
    return;
  }

  // Main mode switch.
  // Only one of these modes runs at a time.
  switch (appMode) {
    case MAIN_MENU:
      handleMenu();
      break;

    case RUN_SORTER:
      runSorter();
      break;

    case SELECT_COLOUR:
      selectColour();
      break;

    case CALIBRATION_MODE:
      if (calibrateColours()) {
        printCalibrationTable();
        showCalDone();
        completionSignal();
      }

      goHome();
      break;

    default:
      goHome();
      break;
  }
}

void printStartup() {
  // Prints controls to Serial Monitor so the user can see the control layout.
  Serial.println(F("=== COLOUR SORTER ==="));
  Serial.println(F("POT A0 = Scroll"));
  Serial.println(F("Top S1 D4 = Select"));
  Serial.println(F("Lower S3 D3 = Back"));
  Serial.println(F("Lower S2 D2 = Manual reject"));
  Serial.println(F("Relay D13 = UI click + completion signal"));
  Serial.println(F("LED D6 = Ready"));
  Serial.println(F("LED D7 = Busy"));
  Serial.println(F("LED D8 = Done"));
}

void showStart() {
  // Startup screen explaining the controls.
  setStatusReady();

  display.clear();
  display.drawString(0, 0, "CONTROLS");
  display.drawString(0, 1, "Pot Scroll");
  display.drawString(0, 2, "S1 Select");
  display.drawString(0, 3, "S3 Back");
  display.drawString(0, 4, "S2 Reject");
  display.drawString(0, 7, "Press S1");
}

void goHome() {
  // Safely return to the main menu.
  // The servo is centred and the relay is turned off.
  appMode = MAIN_MENU;
  sorterState = WAITING_FOR_TOKEN;
  digitalWrite(RELAY_PIN, LOW);
  moveServoSmooth(SENSOR_POS);
  showMenu();
}

void handleMenu() {
  // The potentiometer scrolls through the menu options.
  int newIndex = potToIndex(MENU_ITEMS);

  if (newIndex != menuIndex) {
    menuIndex = newIndex;
    showMenu();
  }

  // S1 confirms the currently highlighted menu option.
  if (buttonPressed(BTN_MAIN)) {
    relayClick();

    if (menuIndex == 0) {
      appMode = RUN_SORTER;
      sorterState = WAITING_FOR_TOKEN;
      sortedThisRun = 0;
    }
    else if (menuIndex == 1) {
      appMode = SELECT_COLOUR;
      showColourSelect();
    }
    else if (menuIndex == 2) {
      appMode = CALIBRATION_MODE;
    }
  }
}

void showMenu() {
  // Main menu spaced across the OLED height so the selected item is clearer.
  setStatusReady();

  display.clear();

  display.drawString(0, 0, "MAIN MENU");

  display.drawString(0, 2, menuIndex == 0 ? "> RUN" : "  RUN");
  display.drawString(0, 4, menuIndex == 1 ? "> COLOUR" : "  COLOUR");
  display.drawString(0, 6, menuIndex == 2 ? "> CALIBRATE" : "  CALIBRATE");
}

void selectColour() {
  // Potentiometer chooses which of the 8 token colours is accepted.
  int newIndex = potToIndex(NUM_TOKEN_COLOURS);

  if (newIndex != acceptedColourIndex) {
    acceptedColourIndex = newIndex;
    showColourSelect();
  }

  // S1 confirms the accepted colour and returns to menu.
  if (buttonPressed(BTN_MAIN)) {
    relayClick();
    goHome();
  }

  delay(100);
}

void showColourSelect() {
  // Displays currently selected accepted colour.
  setStatusReady();

  display.clear();
  display.drawString(0, 0, "ACCEPT:");
  display.drawString(0, 2, colours[acceptedColourIndex].name);
  display.drawString(0, 6, "Pot Scroll");
  display.drawString(0, 7, "S1 Confirm");
}

void runSorter() {
  // Main sorting state machine.
  // It waits for a token, confirms it, reads it, sorts it, then waits for removal.
  int r, g, b;
  const char* tokenColour;

  switch (sorterState) {
    case WAITING_FOR_TOKEN:
      readAveragedRGB(r, g, b);
      tokenColour = classifyColour(r, g, b);

      if (isEmpty(tokenColour)) {
        showInsert();
        moveServoSmooth(SENSOR_POS);
        delay(300);
      } else {
        // First detection only.
        // A second read is required before the token is accepted as present.
        showTokenFound();
        sorterState = CONFIRMING_TOKEN;
      }
      break;

    case CONFIRMING_TOKEN:
      // Short delay lets the token settle before confirming it is really present.
      delay(TOKEN_CONFIRM_DELAY);

      readAveragedRGB(r, g, b);
      tokenColour = classifyColour(r, g, b);

      if (isEmpty(tokenColour)) {
        sorterState = WAITING_FOR_TOKEN;
      } else {
        showReading();
        sorterState = READING_TOKEN;
      }
      break;

    case READING_TOKEN:
      // Longer delay gives the token time to fully settle before final classification.
      delay(SETTLE_DELAY);

      readAveragedRGB(r, g, b);
      tokenColour = classifyColour(r, g, b);

      showDetected(tokenColour);
      sorterState = SORTING_TOKEN;
      break;

    case SORTING_TOKEN:
      // Final classification decides whether token goes right or left.
      readAveragedRGB(r, g, b);
      tokenColour = classifyColour(r, g, b);

      if (strcmp(tokenColour, colours[acceptedColourIndex].name) == 0) {
        acceptedCount++;
        sortedThisRun++;

        correctSound();
        showResult(tokenColour, true);

        moveServoSmooth(RIGHT_CUP);
      } else {
        rejectedCount++;
        sortedThisRun++;

        wrongSound();
        showResult(tokenColour, false);

        moveServoSmooth(LEFT_CUP);
      }

      delay(SORT_DELAY);
      moveServoSmooth(SENSOR_POS);

      // Once the run has sorted the expected number of tokens, signal completion.
      if (sortedThisRun >= TOKENS_PER_RUN) {
        showRunComplete();
        completionSignal();
        sortedThisRun = 0;
        goHome();
        return;
      }

      sorterState = WAITING_FOR_REMOVAL;
      break;

    case WAITING_FOR_REMOVAL:
      // Prevents the same token being counted multiple times.
      // The sorter waits until the sensor sees EMPTY again.
      readAveragedRGB(r, g, b);
      tokenColour = classifyColour(r, g, b);

      if (isEmpty(tokenColour)) {
        sorterState = WAITING_FOR_TOKEN;
      } else {
        showRemove();
        delay(300);
      }
      break;
  }
}

void manualReject() {
  // Manual reject forces the servo to eject left.
  // During Run mode, it counts as one rejected/sorted token.
  // Outside Run mode, it simply clears the mechanism and returns to menu.
  relayClick();

  if (appMode == RUN_SORTER) {
    rejectedCount++;
    sortedThisRun++;
  }

  wrongSound();
  showResult("MANUAL", false);

  moveServoSmooth(LEFT_CUP);
  delay(SORT_DELAY);
  moveServoSmooth(SENSOR_POS);

  if (appMode == RUN_SORTER && sortedThisRun >= TOKENS_PER_RUN) {
    showRunComplete();
    completionSignal();
    sortedThisRun = 0;
    goHome();
    return;
  }

  if (appMode == RUN_SORTER) {
    sorterState = WAITING_FOR_REMOVAL;
  } else {
    goHome();
  }
}

void correctSound() {
  // Short happy chirp for accepted tokens.
  tone(BUZZER_PIN, 1800, 80);
  delay(100);

  tone(BUZZER_PIN, 2200, 80);
  delay(100);

  tone(BUZZER_PIN, 2600, 120);
  delay(140);

  noTone(BUZZER_PIN);
}

void wrongSound() {
  // Lower descending sound for rejected tokens.
  tone(BUZZER_PIN, 500, 250);
  delay(280);

  tone(BUZZER_PIN, 350, 400);
  delay(420);

  noTone(BUZZER_PIN);
}

void completeSound() {
  // Longer rising sound for run complete and calibration complete.
  tone(BUZZER_PIN, 1200, 120);
  delay(150);

  tone(BUZZER_PIN, 1500, 120);
  delay(150);

  tone(BUZZER_PIN, 1800, 120);
  delay(150);

  tone(BUZZER_PIN, 2200, 180);
  delay(220);

  tone(BUZZER_PIN, 2600, 250);
  delay(300);

  noTone(BUZZER_PIN);
}

void relayClick() {
  // Short relay pulse used as tactile/audible UI feedback.
  digitalWrite(RELAY_PIN, HIGH);
  delay(80);
  digitalWrite(RELAY_PIN, LOW);
}

void completionSignal() {
  // Relay clicks three times and then plays the completion sound.
  // Used when calibration is complete and when a full run is complete.
  for (int i = 0; i < 3; i++) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(300);

    digitalWrite(RELAY_PIN, LOW);
    delay(300);
  }

  completeSound();
}

bool calibrateColours() {
  // Steps through all colours and records averaged RGB pulse values.
  // Returns true if calibration finished fully, or false if the user exited early.
  for (int i = 0; i < NUM_COLOURS; i++) {
    setStatusBusy();

    display.clear();
    display.drawString(0, 0, "CALIBRATE");
    display.drawString(0, 2, colours[i].name);
    display.drawString(0, 6, "S1 Save");
    display.drawString(0, 7, "S3 Back");

    while (true) {
      // Manual reject can clear a stuck token during calibration.
      if (buttonPressed(BTN_REJECT)) {
        manualReject();
        return false;
      }

      // Back exits calibration early.
      if (buttonPressed(BTN_BACK)) {
        relayClick();
        return false;
      }

      // S1 saves the current colour reading.
      if (buttonPressed(BTN_MAIN)) {
        relayClick();
        delay(300);

        int r, g, b;
        readAveragedRGB(r, g, b);

        colours[i].r = r;
        colours[i].g = g;
        colours[i].b = b;

        Serial.print(colours[i].name);
        Serial.print(F(" R:"));
        Serial.print(r);
        Serial.print(F(" G:"));
        Serial.print(g);
        Serial.print(F(" B:"));
        Serial.println(b);

        display.clear();
        display.drawString(0, 0, colours[i].name);
        display.drawString(0, 1, "Saved");

        // After saving a real token colour, eject the token left.
        // EMPTY is not ejected because no token should be present.
        if (!isEmpty(colours[i].name)) {
          ejectLeft();
        }

        delay(500);
        break;
      }

      delay(50);
    }
  }

  return true;
}

void showInsert() {
  setStatusReady();

  display.clear();
  display.drawString(0, 0, "INSERT TOKEN");
  display.drawString(0, 2, "Accept:");
  display.drawString(8, 2, colours[acceptedColourIndex].name);
  showCounts();
}

void showTokenFound() {
  setStatusBusy();

  display.clear();
  display.drawString(0, 0, "TOKEN FOUND");
  display.drawString(0, 1, "Confirming...");
  showCounts();
}

void showReading() {
  setStatusBusy();

  display.clear();
  display.drawString(0, 0, "READING...");
  showCounts();
}

void showDetected(const char* colour) {
  setStatusBusy();

  display.clear();
  display.drawString(0, 0, "Detected:");
  display.drawString(0, 1, colour);
  showCounts();
}

void showResult(const char* colour, bool accepted) {
  setStatusBusy();

  display.clear();
  display.drawString(0, 0, colour);
  display.drawString(0, 1, accepted ? "ACCEPTED" : "REJECTED");
  showCounts();
}

void showRemove() {
  setStatusBusy();

  display.clear();
  display.drawString(0, 0, "REMOVE TOKEN");
  showCounts();
}

void showCalDone() {
  setStatusDone();

  display.clear();
  display.drawString(0, 0, "CAL COMPLETE");
  display.drawString(0, 2, "Table printed");
  display.drawString(0, 4, "Relay signal");
}

void showRunComplete() {
  setStatusDone();

  display.clear();
  display.drawString(0, 0, "RUN COMPLETE");
  display.drawString(0, 2, "All tokens");
  display.drawString(0, 3, "sorted");
  display.drawString(0, 5, "Relay signal");
}

void showCounts() {
  // Shows accepted and rejected totals during operation.
  char buf[16];

  sprintf(buf, "A:%d", acceptedCount);
  display.drawString(0, 5, buf);

  sprintf(buf, "R:%d", rejectedCount);
  display.drawString(8, 5, buf);
}

void setStatusReady() {
  // Ready LED shows the machine is waiting or safe to interact with.
  digitalWrite(LED_READY, HIGH);
  digitalWrite(LED_BUSY, LOW);
  digitalWrite(LED_DONE, LOW);
}

void setStatusBusy() {
  // Busy LED shows the machine is reading, sorting, or calibrating.
  digitalWrite(LED_READY, LOW);
  digitalWrite(LED_BUSY, HIGH);
  digitalWrite(LED_DONE, LOW);
}

void setStatusDone() {
  // Done LED shows that calibration or a full run has completed.
  digitalWrite(LED_READY, LOW);
  digitalWrite(LED_BUSY, LOW);
  digitalWrite(LED_DONE, HIGH);
}

void setStatusOff() {
  // Turns all status LEDs off.
  digitalWrite(LED_READY, LOW);
  digitalWrite(LED_BUSY, LOW);
  digitalWrite(LED_DONE, LOW);
}

void ejectLeft() {
  // Used during calibration to clear each token after it has been saved.
  moveServoSmooth(LEFT_CUP);
  delay(700);

  moveServoSmooth(SENSOR_POS);
  delay(700);
}

void moveServoSmooth(int targetAngle) {
  // Moves the servo one degree at a time to avoid throwing tokens too violently.
  int currentAngle = sorterServo.read();

  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      sorterServo.write(pos);
      delay(SERVO_STEP_DELAY);
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      sorterServo.write(pos);
      delay(SERVO_STEP_DELAY);
    }
  }
}

void readAveragedRGB(int &r, int &g, int &b) {
  // Clears previous averages so each colour read uses fresh samples.
  redAverage.clear();
  greenAverage.clear();
  blueAverage.clear();

  // Read red, green, and blue multiple times and average the values.
  for (int i = 0; i < NUM_SAMPLES; i++) {
    redAverage.addValue(getRedPW());
    delay(30);

    greenAverage.addValue(getGreenPW());
    delay(30);

    blueAverage.addValue(getBluePW());
    delay(30);
  }

  r = redAverage.getAverage();
  g = greenAverage.getAverage();
  b = blueAverage.getAverage();
}

const char* classifyColour(int r, int g, int b) {
  // Special-case rules help separate blue and purple, which were close in testing.
  if (r > 1550 && g > 1550 && b > 850 && b < 1050) return "BLUE";

  if (r > 1250 && r < 1500 &&
      g > 1400 && g < 1650 &&
      b > 750 && b < 1000) {
    return "PURPLE";
  }

  // Nearest-match classification.
  // The detected colour is whichever calibrated colour has the smallest RGB distance.
  long bestDistance = 9999999;
  int bestIndex = 0;

  for (int i = 0; i < NUM_COLOURS; i++) {
    long dr = r - colours[i].r;
    long dg = g - colours[i].g;
    long db = b - colours[i].b;

    long distance = dr * dr + dg * dg + db * db;

    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = i;
    }
  }

  return colours[bestIndex].name;
}

int getRedPW() {
  // Select red filter on the colour sensor and measure pulse width.
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, LOW);
  delay(5);
  return pulseIn(SENSOR_OUT, LOW, 50000);
}

int getGreenPW() {
  // Select green filter on the colour sensor and measure pulse width.
  digitalWrite(SENSOR_S2, HIGH);
  digitalWrite(SENSOR_S3, HIGH);
  delay(5);
  return pulseIn(SENSOR_OUT, LOW, 50000);
}

int getBluePW() {
  // Select blue filter on the colour sensor and measure pulse width.
  digitalWrite(SENSOR_S2, LOW);
  digitalWrite(SENSOR_S3, HIGH);
  delay(5);
  return pulseIn(SENSOR_OUT, LOW, 50000);
}

bool buttonPressed(int pin) {
  // Simple debounce for active-HIGH buttons.
  // Waits until the button is released before returning true.
  if (digitalRead(pin) == HIGH) {
    delay(25);

    if (digitalRead(pin) == HIGH) {
      while (digitalRead(pin) == HIGH) {
        delay(10);
      }
      return true;
    }
  }

  return false;
}

int potToIndex(int numberOfItems) {
  // Converts potentiometer value from 0-1023 into a menu/colour index.
  int value = analogRead(POT_PIN);
  int index = (long)value * numberOfItems / 1024;

  if (index < 0) {
    index = 0;
  }

  if (index >= numberOfItems) {
    index = numberOfItems - 1;
  }

  return index;
}

bool isEmpty(const char* colourName) {
  // Returns true when classifier says there is no token present.
  return strcmp(colourName, "EMPTY") == 0;
}

void printCalibrationTable() {
  // Prints a ready-to-copy colour calibration table to Serial Monitor.
  Serial.println();
  Serial.println(F("=== FULL CALIBRATION READOUT ==="));
  Serial.println(F("ColourCalibration colours[NUM_COLOURS] = {"));

  for (int i = 0; i < NUM_COLOURS; i++) {
    Serial.print(F("  {\""));
    Serial.print(colours[i].name);
    Serial.print(F("\", "));
    Serial.print(colours[i].r);
    Serial.print(F(", "));
    Serial.print(colours[i].g);
    Serial.print(F(", "));
    Serial.print(colours[i].b);
    Serial.print(F("}"));

    if (i < NUM_COLOURS - 1) {
      Serial.print(F(","));
    }

    Serial.println();
  }

  Serial.println(F("};"));
  Serial.println();
}