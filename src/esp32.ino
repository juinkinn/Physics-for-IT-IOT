#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>   // Correct library for ESP32

#define DHTPIN 4            // DHT11 pin (temperature and humidity)
#define DHTTYPE DHT11       // DHT11 type
#define POTPIN 35           // Potentiometer pin (for controlling servo)
#define PIRPIN 32           // PIR motion sensor pin
#define BUZZERPIN 23        // Buzzer pin for motion detection alert
#define SERVOPIN 25         // Servo motor control pin (PWM)
#define RELAYPIN 22         // Relay pin for controlling servo power
#define LIGHTPIN 34         // Light sensor pin (analog input)
#define LEDPIN 27           // LED pin (digital output)

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD address, adjust to match your device
Servo wateringServo;

int pirState = LOW;
unsigned long lastServoRunTime = 0;      // Stores the last time the servo was activated
unsigned long lastServoUpdateTime = 0;   // Tracks time for servo oscillation steps
const unsigned long servoInterval = 300;  // 1 hour in milliseconds
const unsigned long servoRunDuration = 300000;  // 5 minutes in milliseconds
const unsigned long servoStepDelay = 100;  // Delay between servo movements (in ms)
bool servoRunning = false;
bool servoDirectionUp = true;  // Direction of servo movement (up or down)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize DHT11 sensor
  dht.begin();

  // Initialize Servo motor
  wateringServo.attach(SERVOPIN);

  // Set pin modes for PIR sensor, Buzzer, Relay, and LED
  pinMode(PIRPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  // Initial state of Relay, Buzzer, and LED is OFF
  digitalWrite(RELAYPIN, LOW);
  digitalWrite(BUZZERPIN, LOW);
  digitalWrite(LEDPIN, HIGH);
}

void loop() {
  // Read temperature and humidity from the DHT11 sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Check if the DHT11 readings are valid (not NaN)
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return; // Exit if sensor reading failed
  }

  // Read potentiometer value and map it to a servo angle (0 to 180 degrees)
  int potValue = analogRead(POTPIN);

  // Servo motor logic: Run every 1 hour for 5 minutes unless halted by high potentiometer value
  unsigned long currentTime = millis();
  if (currentTime - lastServoRunTime >= servoInterval && !servoRunning) {
    servoRunning = true;
    lastServoRunTime = currentTime; // Update the last run time
    digitalWrite(RELAYPIN, HIGH);   // Turn ON the relay to enable servo power
    Serial.println("Starting servo motor operation...");
  }

  if (servoRunning) {
    if (potValue > 350) {  // Halt if potentiometer value is "huge"
      servoRunning = false;
      digitalWrite(RELAYPIN, LOW);  // Turn OFF the relay to disable servo power
      Serial.println("Servo halted due to high potentiometer value.");
    } else if (currentTime - lastServoRunTime >= servoRunDuration) {
      servoRunning = false; // Stop the servo after 5 minutes
      digitalWrite(RELAYPIN, LOW);  // Turn OFF the relay to disable servo power
      Serial.println("Servo operation completed.");
    } else {
      // Oscillate the servo between 0° and 180°
      if (currentTime - lastServoUpdateTime >= servoStepDelay) {
        lastServoUpdateTime = currentTime;
        static int currentAngle = 0; // Static variable to track current angle

        if (servoDirectionUp) {
          currentAngle += 10; // Increase angle
          if (currentAngle >= 180) {
            currentAngle = 180;
            servoDirectionUp = false; // Change direction to down
          }
        } else {
          currentAngle -= 10; // Decrease angle
          if (currentAngle <= 0) {
            currentAngle = 0;
            servoDirectionUp = true; // Change direction to up
          }
        }

        wateringServo.write(currentAngle); // Set the servo position
        Serial.print("Servo Angle: ");
        Serial.println(currentAngle);
      }
    }
  } else {
    wateringServo.write(0); // Ensure servo is in a default state when not running
    digitalWrite(RELAYPIN, LOW); // Keep the relay OFF to save power
  }

  // Read light sensor value
  int lightLevel = analogRead(LIGHTPIN);

  // Turn on LED if light level is low
  int lightThreshold = 100; // Define a threshold for low light (adjust as needed)
  if (lightLevel < lightThreshold) {
    digitalWrite(LEDPIN, HIGH); // Turn on the LED
    Serial.println("Low light detected. LED turned ON.");
  } else {
    digitalWrite(LEDPIN, LOW); // Turn off the LED
    Serial.println("Sufficient light detected. LED turned OFF.");
  }

  // Display temperature and humidity on the LCD screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(temperature) + " C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + String(humidity) + " %");
  
  // Motion detection with PIR sensor
  int pirValue = digitalRead(PIRPIN);  // Read PIR sensor state
  if (pirValue == HIGH) {              // If motion is detected
    digitalWrite(BUZZERPIN, HIGH);     // Turn on the buzzer
    if (pirState == LOW) {             // Only print on state change
      Serial.println("Motion detected!");
      pirState = HIGH;
    }
  } else {                             // If no motion is detected
    digitalWrite(BUZZERPIN, LOW);      // Turn off the buzzer
    if (pirState == HIGH) {            // Only print on state change
      Serial.println("Motion ended!");
      pirState = LOW;
    }
  }

  // Print data to the serial monitor for debugging
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Potentiometer: ");
  Serial.print(potValue);
  Serial.print(", Light Level: ");
  Serial.println(lightLevel);

  delay(100);  // Delay for 100ms for stable updates
}
