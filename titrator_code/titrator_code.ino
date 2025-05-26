#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define ACID_VALVE_PIN 12
#define BASE_VALVE_PIN 13
#define PH_SENSOR_PIN A0

// Encoder as three separate buttons
#define ENCODER_CLK 6  // Use as UP button
#define ENCODER_DT 5   // Use as DOWN button
#define ENCODER_SW 2   // Use as SELECT button

// pH calibration values
#define PH_SAMPLES 10
#define NEUTRAL_PH 7.0

// LCD setup (address, columns, rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Button states
bool upPressed = false;
bool downPressed = false;
bool selectPressed = false;
unsigned long lastButtonPress = 0;
#define DEBOUNCE_TIME 300

// Menu states
enum MenuState {
  MENU_MAIN,
  MENU_SHOW_PH,
  MENU_NEUTRALIZE
};

MenuState currentState = MENU_MAIN;
int menuSelection = 0;

// Function prototypes
float readPH();
void neutralizeSolution();
void checkButtons();
void updateDisplay();

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("Titrator starting...");
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize pins
  pinMode(ACID_VALVE_PIN, OUTPUT);
  pinMode(BASE_VALVE_PIN, OUTPUT);
  
  // Set up button pins with pull-up resistors
  pinMode(ENCODER_CLK, INPUT_PULLUP);  // UP
  pinMode(ENCODER_DT, INPUT_PULLUP);   // DOWN
  pinMode(ENCODER_SW, INPUT_PULLUP);   // SELECT
  
  // Ensure valves are closed initially
  digitalWrite(ACID_VALVE_PIN, LOW);
  digitalWrite(BASE_VALVE_PIN, LOW);
  
  // Display welcome message
  lcd.setCursor(0, 0);
  lcd.print("Titrator System");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(2000);
  
  // Show menu
  updateDisplay();
}

void loop() {
  checkButtons();
  
  if (currentState == MENU_SHOW_PH) {
    // Update pH reading every 500ms
    static unsigned long lastPHUpdate = 0;
    if (millis() - lastPHUpdate > 500) {
      float pH = readPH();
      lcd.setCursor(0, 1);
      lcd.print("pH: ");
      lcd.print(pH, 2);
      lcd.print("       ");
      lastPHUpdate = millis();
    }
  }
  else if (currentState == MENU_NEUTRALIZE) {
    neutralizeSolution();
    // Return to main menu after neutralization
    currentState = MENU_MAIN;
    updateDisplay();
  }
}

float readPH() {
  // Take multiple samples to improve accuracy
  int sum = 0;
  for (int i = 0; i < PH_SAMPLES; i++) {
    sum += analogRead(PH_SENSOR_PIN);
    delay(10);
  }
  int average = sum / PH_SAMPLES;
  
  // Convert analog reading to pH value
  float voltage = average * (5.0 / 1023.0);
  float phValue = 3.5 * voltage;
  
  Serial.print("pH: ");
  Serial.println(phValue, 2);
  
  return phValue;
}

void neutralizeSolution() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Neutralizing...");
  
  float pH;
  bool neutralized = false;
  
  while (!neutralized) {
    pH = readPH();
    
    lcd.setCursor(0, 1);
    lcd.print("pH: ");
    lcd.print(pH, 2);
    lcd.print("       ");
    
    if (pH < NEUTRAL_PH - 0.1) {
      // Solution is acidic, add base
      lcd.setCursor(12, 1);
      lcd.print("BASE");
      digitalWrite(ACID_VALVE_PIN, LOW);
      digitalWrite(BASE_VALVE_PIN, HIGH);
      delay(200);  // Add base for 200ms
      digitalWrite(BASE_VALVE_PIN, LOW);
      delay(1000);  // Wait for mixing
    }
    else if (pH > NEUTRAL_PH + 0.1) {
      // Solution is basic, add acid
      lcd.setCursor(12, 1);
      lcd.print("ACID");
      digitalWrite(BASE_VALVE_PIN, LOW);
      digitalWrite(ACID_VALVE_PIN, HIGH);
      delay(200);  // Add acid for 200ms
      digitalWrite(ACID_VALVE_PIN, LOW);
      delay(1000);  // Wait for mixing
    }
    else {
      // Solution is neutral
      lcd.setCursor(0, 1);
      lcd.print("Neutralized!    ");
      neutralized = true;
      delay(2000);
    }
    
    // Check if select button is pressed to abort
    if (digitalRead(ENCODER_SW) == LOW && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
      lastButtonPress = millis();
      digitalWrite(ACID_VALVE_PIN, LOW);
      digitalWrite(BASE_VALVE_PIN, LOW);
      lcd.setCursor(0, 1);
      lcd.print("Aborted!        ");
      delay(1000);
      neutralized = true;
    }
  }
  
  // Ensure valves are closed
  digitalWrite(ACID_VALVE_PIN, LOW);
  digitalWrite(BASE_VALVE_PIN, LOW);
}

void checkButtons() {
  // Check UP button (CLK)
  if (digitalRead(ENCODER_CLK) == LOW && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
    lastButtonPress = millis();
    
    if (currentState == MENU_MAIN) {
      menuSelection = 0;  // Set to "Neutralize"
      updateDisplay();
    }
    
  }
  
  // Check DOWN button (DT)
  if (digitalRead(ENCODER_DT) == LOW && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
    lastButtonPress = millis();
    if (currentState == MENU_MAIN) {
      menuSelection = 1;  // Set to "Show pH"
      updateDisplay();
    }
  }
  
  // Check SELECT button (SW)
  if (digitalRead(ENCODER_SW) == LOW && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
      lastButtonPress = millis();
      
      if (currentState == MENU_MAIN) {
        // Handle menu selection
        if (menuSelection == 0) {
          currentState = MENU_NEUTRALIZE;
        } else {
          currentState = MENU_SHOW_PH;
        }
        updateDisplay();
      } 
      else if (currentState == MENU_SHOW_PH) {
        // Return to main menu
        currentState = MENU_MAIN;
        updateDisplay();
      }
    
  }
}

void updateDisplay() {
  lcd.clear();
  
  switch (currentState) {
    case MENU_MAIN:
      lcd.setCursor(0, 0);
      lcd.print("Menu:");
      lcd.setCursor(0, 1);
      if (menuSelection == 0) {
        lcd.print("> Neutralize");
      } else {
        lcd.print("> Show pH");
      }
      break;
      
    case MENU_SHOW_PH:
      lcd.setCursor(0, 0);
      lcd.print("pH Measurement");
      // pH value will be updated in loop()
      break;
      
    case MENU_NEUTRALIZE:
      // Display will be handled in neutralizeSolution()
      break;
  }
}
