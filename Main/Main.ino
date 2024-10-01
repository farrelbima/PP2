#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// I2C declaration
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns, 2 rows

// Pin assignments
#define RELAY_1 23  // Heating element relay
#define RELAY_2 22  // Motor relay
#define DS18B20_PIN 21
#define PH_SENSOR_PIN 34  // Analog input for pH sensor

// Potentiometer and PWM pins
#define POT_PIN 34  // Potentiometer connected to GPIO34 (Analog input)
#define PWM_PIN 18  // PWM output on GPIO18 (change to your desired PWM pin)

// Relay states
bool heatingOn = false;
bool motorOn = false;

// DS18B20 Temperature Sensor
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
float temperature = 0.0;

// Keypad Setup
const byte ROW_NUM = 4;
const byte COLUMN_NUM = 4;
byte pin_rows[ROW_NUM] = {19, 18, 5, 17};
byte pin_column[COLUMN_NUM] = {16, 4, 0, 2};
char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

// Variables for timing
unsigned long heatingStartTime = 0;
unsigned long motorStartTime = 0;
unsigned long heatingDuration = 0;
unsigned long motorDuration = 0;
bool heatingTimerSet = false;
bool motorTimerSet = false;

const unsigned long eventTime_1_Heat = 1000;
const unsigned long eventTime_2_pH = 1000;
const unsigned long eventTime_3_pot = 100;

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;

// PWM setup
int potValue = 0;      // Variable to store the potentiometer reading (0 to 4095)
int dutyCycle = 0;     // Duty cycle (0 to 100)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize temperature sensor
  sensors.begin();

  // Initialize relay pins
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  digitalWrite(RELAY_1, LOW);  // Ensure relays are off at the start
  digitalWrite(RELAY_2, LOW);

  // Initialize I2C LCD (0x27, 16x2)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();

  // PWM setup for ESP32
  ledcSetup(0, 50000, 8);  // PWM channel 0, frequency 50kHz, resolution 8-bit
  ledcAttachPin(PWM_PIN, 0);  // Attach PWM to GPIO18
}

void loop() {
  // Millis
  unsigned long currentTime = millis();
  
  // Check keypad for input
  char key = keypad.getKey();
  
  if (key) {
    handleKeyPress(key);
  }

  // Handle automatic shutoff of the heating element
  if (heatingOn && heatingTimerSet) {
    if (millis() - heatingStartTime >= heatingDuration) {
      heatingOn = false;
      digitalWrite(RELAY_1, LOW);  // Turn off heating element
      heatingTimerSet = false;
    }
  }

  // Handle automatic shutoff of the motor
  if (motorOn && motorTimerSet) {
    if (millis() - motorStartTime >= motorDuration) {
      motorOn = false;
      digitalWrite(RELAY_2, LOW);  // Turn off motor
      motorTimerSet = false;
    }
  }

  // Update temperature sensor
  if( currentTime - previousTime_1 >= eventTime_1_Heat ){
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    previousTime_1 = currentTime;
  }
 
  // Turn off heating element if temperature exceeds threshold
  if (temperature > 30.0) {
    heatingOn = false;
    digitalWrite(RELAY_1, LOW);  // Turn off heating element relay
    heatingTimerSet = false;  // Cancel the timer
  }

  // Read pH sensor value and output via Serial
  if( currentTime - previousTime_2 >= eventTime_2_pH ){
    int phValue = analogRead(PH_SENSOR_PIN);
    float pH = (phValue / 4095.0) * 14.0;  // Convert to pH scale (0-14)
    
    Serial.print("pH Value: ");
    Serial.println(pH);

    previousTime_2 = currentTime;
  }

 // Potentiometer PWM update
 if( currentTime - previousTime_3 >= eventTime_3_pot ){
    // Read the potentiometer value (0 to 4095)
    potValue = analogRead(POT_PIN);

    // Map the potentiometer value to duty cycle (0 to 100%)
    dutyCycle = map(potValue, 0, 4095, 0, 100);

    // Set the PWM duty cycle based on the potentiometer input
    ledcWrite(0, (dutyCycle * 255) / 100);  // Scale the duty cycle to match 8-bit resolution (0-255)

    previousTime_3 = currentTime;
 }
}

void handleKeyPress(char key) {
  switch (key) {
    case 'A':
      heatingOn = !heatingOn;
      if (heatingOn) {
        heatingStartTime = millis();
        heatingDuration = setTimer();  // Get timer duration from the keypad
        heatingTimerSet = heatingDuration > 0;  // Set the timer only if a duration was entered
        digitalWrite(RELAY_1, HIGH);  // Turn on heating element
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Heating ON");
      } else {
        digitalWrite(RELAY_1, LOW);  // Turn off heating element
        heatingTimerSet = false;  // Cancel the timer if manually turned off
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Heating OFF");
      }
      break;

    case 'B':
      motorOn = !motorOn;
      if (motorOn) {
        motorStartTime = millis();
        motorDuration = setTimer();  // Get timer duration from the keypad
        motorTimerSet = motorDuration > 0;  // Set the timer only if a duration was entered
        digitalWrite(RELAY_2, HIGH);  // Turn on motor
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Motor ON");
      } else {
        digitalWrite(RELAY_2, LOW);  // Turn off motor
        motorTimerSet = false;  // Cancel the timer if manually turned off
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Motor OFF");
      }
      break;

    case 'C':
      heatingOn = false;
      digitalWrite(RELAY_1, LOW);  // Turn off heating element
      heatingTimerSet = false;  // Cancel the timer
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Heating OFF");
      break;

    case 'D':
      motorOn = false;
      digitalWrite(RELAY_2, LOW);  // Turn off motor
      motorTimerSet = false;  // Cancel the timer
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Motor OFF");
      break;

    default:
      // Handle invalid key press
      break;
  }
}

// Function to set timer using the keypad (returns duration in milliseconds)
unsigned long setTimer() {
  String input = "";
  char key;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Time (seconds):");
  lcd.setCursor(0, 1);
  
  // Collect numerical input from the keypad
  while (true) {
    key = keypad.getKey();
    if (key && key >= '0' && key <= '9') {
      input += key;
      lcd.print(key);  // Display the number on the LCD as it is typed
    }
    
    // User presses '#' to confirm the input
    if (key == '#') {
      break;
    }
  }
  
  // Convert the input string to a number (duration in seconds)
  unsigned long seconds = input.toInt();
  
  // Return the duration in milliseconds
  return seconds * 1000;
}
