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

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x3F, 16, 2);  // LCD address, adjust to match your device
Servo wateringServo;
int val = 0;
int pirState = LOW;

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
  
  // Set pin modes for PIR sensor and Buzzer
  pinMode(PIRPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
  
  // Initial state of Buzzer is OFF
  digitalWrite(BUZZERPIN, LOW);
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
  int servoAngle = map(potValue, 0, 4095, 0, 180);  // Map potentiometer range to servo angle
  wateringServo.write(servoAngle); // Adjust the servo position based on potentiometer

  // Display temperature and humidity on the LCD screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(temperature) + " C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + String(humidity) + " %");
  
  // Motion detection with PIR sensor
  val = digitalRead(PIRPIN);  // Read PIR sensor state
  if (val == HIGH) {           // If motion is detected
    digitalWrite(BUZZERPIN, HIGH);  // Turn on the buzzer
    if (pirState == LOW) {     // Only print on state change
      Serial.println("Motion detected!");
      pirState = HIGH;
    }
  } else {                     // If no motion is detected
    digitalWrite(BUZZERPIN, LOW);   // Turn off the buzzer
    if (pirState == HIGH) {    // Only print on state change
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
  Serial.print(" (Servo Angle: ");
  Serial.print(servoAngle);
  Serial.println(")");

  // Wait for a second before repeating
  delay(1000);  // Delay for 1 second
}