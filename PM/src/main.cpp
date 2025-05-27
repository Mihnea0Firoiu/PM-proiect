#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

// Pin definitions (converted to port/pin format)
#define ACID_VALVE_PIN 4    // PB4 (pin 12)
#define BASE_VALVE_PIN 5    // PB5 (pin 13)
#define PH_SENSOR_PIN 0     // PC0 (A0)

// Encoder as three separate buttons
#define ENCODER_CLK 6       // PD6 (pin 6)
#define ENCODER_DT 5        // PD5 (pin 5)
#define ENCODER_SW 2        // PD2 (pin 2)

// pH calibration values
#define PH_SAMPLES 10
#define NEUTRAL_PH 7.0

// I2C LCD constants
#define LCD_ADDR 0x27
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_DATA 0xF0
#define LCD_CMD 0x00

// Button debounce
#define DEBOUNCE_TIME 300

// Menu states
enum MenuState {
  MENU_MAIN,
  MENU_SHOW_PH,
  MENU_NEUTRALIZE
};

MenuState currentState = MENU_MAIN;
int menuSelection = 0;
unsigned long lastButtonPress = 0;
volatile unsigned long systemTime = 0;

// Function prototypes
void initSystem();
void initADC();
void initI2C();
void initTimer();
float readPH();
void neutralizeSolution();
void checkButtons();
void updateDisplay();
void lcdInit();
void lcdPrint(const char* str);
void lcdSetCursor(int col, int row);
void lcdClear();
void lcdBacklight();
void i2cStart();
void i2cStop();
void i2cWrite(uint8_t data);
void lcdWrite4bits(uint8_t value);
void lcdCommand(uint8_t cmd);
void lcdData(uint8_t data);
uint16_t analogRead(uint8_t pin);
unsigned long millis();

// Timer interrupt for millis() function
ISR(TIMER0_OVF_vect) {
    systemTime++;
}

void floatToString(float value, char *buffer, uint8_t precision) {
    // Extract the integer part
    int intPart = (int)value;
    
    // Convert integer part to string
    int i = 0;
    if (intPart == 0) {
        buffer[i++] = '0';
    } else {
        // Handle negative numbers
        if (value < 0 && intPart == 0) {
            buffer[i++] = '-';
        }
        
        // Convert integer part digit by digit
        int tempInt = (value < 0) ? -intPart : intPart;
        int reverseIndex = i;
        while (tempInt > 0) {
            buffer[i++] = '0' + (tempInt % 10);
            tempInt /= 10;
        }
        
        // Reverse the integer digits
        int j = reverseIndex;
        int k = i - 1;
        while (j < k) {
            char temp = buffer[j];
            buffer[j++] = buffer[k];
            buffer[k--] = temp;
        }
    }
    
    // Add decimal point
    buffer[i++] = '.';
    
    // Extract and convert fractional part
    float fractPart = (value < 0) ? -value - (-intPart) : value - intPart;
    for (uint8_t j = 0; j < precision; j++) {
        fractPart *= 10;
        int digit = (int)fractPart;
        buffer[i++] = '0' + digit;
        fractPart -= digit;
    }
    
    // Null terminate the string
    buffer[i] = '\0';
}

int main() {
    initSystem();
    
    // Display welcome message
    lcdSetCursor(0, 0);
    lcdPrint("Titrator System");
    lcdSetCursor(0, 1);
    lcdPrint("Starting...");
    _delay_ms(2000);
    
    // Show menu
    updateDisplay();
    
    while(1) {
        checkButtons();
        
        if (currentState == MENU_SHOW_PH) {
            // Update pH reading every 500ms
            static unsigned long lastPHUpdate = 0;
            if (millis() - lastPHUpdate > 500) {
                float pH = readPH();
                lcdSetCursor(0, 1);
                lcdPrint("pH: ");
                // Convert float to string and print
                char buffer[10];
                floatToString(pH, buffer, 2);
                lcdPrint(buffer);
                lcdPrint("       ");
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
    
    return 0;
}

void initSystem() {

    // Initialize ADC
    initADC();
    
    // Initialize I2C
    initI2C();
    
    // Initialize LCD
    lcdInit();
    lcdBacklight();
    
    // Initialize timer for millis()
    initTimer();
    
    // Set up valve pins as outputs
    DDRB |= (1 << ACID_VALVE_PIN) | (1 << BASE_VALVE_PIN);
    
    // Set up button pins as inputs with pull-ups
    DDRD &= ~((1 << ENCODER_CLK) | (1 << ENCODER_DT) | (1 << ENCODER_SW));
    PORTD |= (1 << ENCODER_CLK) | (1 << ENCODER_DT) | (1 << ENCODER_SW);
    
    // Ensure valves are closed initially
    PORTB &= ~((1 << ACID_VALVE_PIN) | (1 << BASE_VALVE_PIN));
    
    // Enable global interrupts
    sei();
}

void initADC() {
    // Set reference voltage to AVCC
    ADMUX = (1 << REFS0);
    // Enable ADC, set prescaler to 128 for 16MHz/128 = 125kHz
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void initI2C() {
    // Set SCL frequency to 100kHz with 16MHz CPU
    TWBR = 72;
    TWSR = 0;
}

void initTimer() {
    // Configure Timer0 for millis() function
    TCCR0A = 0;
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    TIMSK0 = (1 << TOIE0); // Enable overflow interrupt
}

unsigned long millis() {
    unsigned long ms;
    uint8_t oldSREG = SREG;
    cli();
    ms = systemTime;
    SREG = oldSREG;
    return ms;
}

uint16_t analogRead(uint8_t pin) {
    // Set the analog pin
    ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while(ADCSRA & (1 << ADSC));
    return ADC;
}

// I2C Functions
void i2cStart() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
}

void i2cStop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2cWrite(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
}

// LCD Functions
void lcdWrite4bits(uint8_t value) {
    i2cStart();
    i2cWrite(LCD_ADDR << 1);
    i2cWrite(value | LCD_BACKLIGHT);
    i2cStop();
    _delay_us(1);
    
    // Pulse enable
    i2cStart();
    i2cWrite(LCD_ADDR << 1);
    i2cWrite(value | LCD_ENABLE | LCD_BACKLIGHT);
    i2cStop();
    _delay_us(1);
    
    i2cStart();
    i2cWrite(LCD_ADDR << 1);
    i2cWrite(value | LCD_BACKLIGHT);
    i2cStop();
    _delay_us(50);
}

void lcdCommand(uint8_t cmd) {
    lcdWrite4bits(cmd & 0xF0);
    lcdWrite4bits((cmd << 4) & 0xF0);
}

void lcdData(uint8_t data) {
    lcdWrite4bits((data & 0xF0) | 0x01);
    lcdWrite4bits(((data << 4) & 0xF0) | 0x01);
}

void lcdInit() {
    _delay_ms(50);
    
    // Initialize in 4-bit mode
    lcdWrite4bits(0x30);
    _delay_ms(5);
    lcdWrite4bits(0x30);
    _delay_us(100);
    lcdWrite4bits(0x30);
    _delay_us(100);
    lcdWrite4bits(0x20);
    _delay_us(100);
    
    // Function set: 4-bit, 2 lines, 5x8 font
    lcdCommand(0x28);
    // Display on, cursor off, blink off
    lcdCommand(0x0C);
    // Clear display
    lcdCommand(0x01);
    _delay_ms(2);
    // Entry mode: increment cursor
    lcdCommand(0x06);
}

void lcdClear() {
    lcdCommand(0x01);
    _delay_ms(2);
}

void lcdSetCursor(int col, int row) {
    uint8_t address = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcdCommand(address);
}

void lcdPrint(const char* str) {
    while(*str) {
        lcdData(*str++);
    }
}

void lcdBacklight() {
    i2cStart();
    i2cWrite(LCD_ADDR << 1);
    i2cWrite(LCD_BACKLIGHT);
    i2cStop();
}

float readPH() {
    // Take multiple samples to improve accuracy
    long sum = 0;
    for (int i = 0; i < PH_SAMPLES; i++) {
        sum += analogRead(PH_SENSOR_PIN);
        _delay_ms(10);
    }
    int average = sum / PH_SAMPLES;

    float calibph7 = 2.9; // Voltage reading when probe is in pH 7.01 buffer
    float calibph4 = 3.3; // Voltage reading when probe is in pH 4.01 buffer

    float m;
    float b;

    m = (4.01 - 7.01) / (calibph4 - calibph7);
    b = 7.01 - m * calibph7;

    // Convert analog reading to pH value
    float voltage = average * (4.64 / 1024.0);
    float phValue = m * voltage + b;
    lcdSetCursor(0, 1);
    lcdPrint("pH: ");

    char buffer[10];
    floatToString(phValue, buffer, 2);
    lcdPrint(buffer);
    
    return phValue;
}

void neutralizeSolution() {
    lcdClear();
    lcdSetCursor(0, 0);
    lcdPrint("Neutralizing...");
    
    float pH;
    bool neutralized = false;
    
    while (!neutralized) {
        pH = readPH();
        
        lcdSetCursor(0, 1);
        lcdPrint("pH: ");
        char buffer[10];
        floatToString(pH, buffer, 2);  // Format with 2 decimal places
        lcdPrint(buffer);
        lcdPrint("       ");
        
        if (pH < NEUTRAL_PH - 0.1) {
            // Solution is acidic, add base
            lcdSetCursor(12, 1);
            lcdPrint("BASE");
            PORTB &= ~(1 << ACID_VALVE_PIN);
            PORTB |= (1 << BASE_VALVE_PIN);
            _delay_ms(200);  // Add base for 200ms
            PORTB &= ~(1 << BASE_VALVE_PIN);
            _delay_ms(1000);  // Wait for mixing
        }
        else if (pH > NEUTRAL_PH + 0.1) {
            // Solution is basic, add acid
            lcdSetCursor(12, 1);
            lcdPrint("ACID");
            PORTB &= ~(1 << BASE_VALVE_PIN);
            PORTB |= (1 << ACID_VALVE_PIN);
            _delay_ms(200);  // Add acid for 200ms
            PORTB &= ~(1 << ACID_VALVE_PIN);
            _delay_ms(1000);  // Wait for mixing
        }
        else {
            // Solution is neutral
            lcdSetCursor(0, 1);
            lcdPrint("Neutralized!    ");
            neutralized = true;
            _delay_ms(2000);
        }
        
        // Check if select button is pressed to abort
        if (!(PIND & (1 << ENCODER_SW)) && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
            lastButtonPress = millis();
            PORTB &= ~((1 << ACID_VALVE_PIN) | (1 << BASE_VALVE_PIN));
            lcdSetCursor(0, 1);
            lcdPrint("Aborted!        ");
            _delay_ms(1000);
            neutralized = true;
        }
    }
    
    // Ensure valves are closed
    PORTB &= ~((1 << ACID_VALVE_PIN) | (1 << BASE_VALVE_PIN));
}

void checkButtons() {
    // Check UP button (CLK)
    if (!(PIND & (1 << ENCODER_CLK)) && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
        lastButtonPress = millis();
        
        if (currentState == MENU_MAIN) {
            menuSelection = 0;  // Set to "Neutralize"
            updateDisplay();
        }
    }
    
    // Check DOWN button (DT)
    if (!(PIND & (1 << ENCODER_DT)) && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
        lastButtonPress = millis();
        if (currentState == MENU_MAIN) {
            menuSelection = 1;  // Set to "Show pH"
            updateDisplay();
        }
    }
    
    // Check SELECT button (SW)
    if (!(PIND & (1 << ENCODER_SW)) && (millis() - lastButtonPress > DEBOUNCE_TIME)) {
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
    lcdClear();
    
    switch (currentState) {
        case MENU_MAIN:
            lcdSetCursor(0, 0);
            lcdPrint("Menu:");
            lcdSetCursor(0, 1);
            if (menuSelection == 0) {
                lcdPrint("> Neutralize");
            } else {
                lcdPrint("> Show pH");
            }
            break;
            
        case MENU_SHOW_PH:
            lcdSetCursor(0, 0);
            lcdPrint("pH Measurement");
            // pH value will be updated in main loop
            break;
            
        case MENU_NEUTRALIZE:
            // Display will be handled in neutralizeSolution()
            break;
    }
}
