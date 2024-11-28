#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>   
#include <WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 4            // DHT22 pin (temperature and humidity)
#define DHTTYPE DHT22       
#define POTPIN 35           // Potentiometer pin (for controlling servo)
#define PIRPIN 32           // PIR motion sensor pin
#define BUZZERPIN 23        // Buzzer pin 
#define SERVOPIN 25         // Servo motor control pin (PWM)
#define RELAYPIN 22         // Relay pin for controlling servo power
#define LIGHTPIN 34         // Light sensor pin (analog input)
#define LEDPIN 27           // LED pin (digital output)

const char* ssid = "Wokwi-GUEST";
const char* password = "";

//***Set server***
const char* mqttServer = "broker.hivemq.com"; 
int port = 1883;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD address
Servo wateringServo;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int pirState = LOW;
unsigned long lastServoRunTime = 0;      // Stores the last time the servo was activated
unsigned long lastServoUpdateTime = 0;   // Tracks time for servo oscillation steps
const unsigned long servoInterval = 3000;  // Every 3s servo runs
const unsigned long servoRunDuration = 30000;  // 30s, Running duration of servo
const unsigned long servoStepDelay = 10;  // Delay between servo movements (in ms)
bool servoRunning = false;
bool servoDirectionUp = true;  // Direction of servo movement (up or down)

const float GAMMA = 0.7;       // Gamma value for the photoresistor
const float RL10 = 50;         // Resistance of LDR at 10 lux in kilo-ohms

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("/wateringTKMQ/temp");
      mqttClient.subscribe("/wateringTKMQ/humid");
     
    }
    else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

//MQTT Receiver
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);

  //***Code here to process the received package***

}

void setup() {
  Serial.begin(115200);
  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );

  lcd.init();
  lcd.backlight();

  dht.begin();

  wateringServo.attach(SERVOPIN);

  pinMode(PIRPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  digitalWrite(RELAYPIN, LOW);
  digitalWrite(BUZZERPIN, LOW);
  digitalWrite(LEDPIN, HIGH);
}

void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  // Read temperature and humidity from the DHT11 sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Publish temperature and humidity to MQTT
  char tempBuffer[50];
  char humidBuffer[50];

  // Convert temperature and humidity to string
  snprintf(tempBuffer, sizeof(tempBuffer), "%.2f", temperature);
  snprintf(humidBuffer, sizeof(humidBuffer), "%.2f", humidity);

  // Publish to respective topics
  mqttClient.publish("/wateringTKMQ/temp", tempBuffer);
  Serial.println("Published temperature data.");

  mqttClient.publish("/wateringTKMQ/humid", humidBuffer);
  Serial.println("Published humidity data.");


  // Display temperature, humidity, and light levels on the LCD screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(temperature) + " C");
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("Humid: " + String(humidity) + " %");
  delay(2000);

  // Read potentiometer value and map it to a servo angle (0 to 180 degrees)
  int potValue = analogRead(POTPIN);

  unsigned long currentTime = millis();
  if (currentTime - lastServoRunTime >= servoInterval && !servoRunning) {
    servoRunning = true;
    lastServoRunTime = currentTime; // Update the last run time
    digitalWrite(RELAYPIN, HIGH);   // Turn ON the relay to enable servo power
    Serial.println("Starting servo motor operation...");
  }

  if (servoRunning) {
    if (potValue > 350) {  // Halt if potentiometer value is high (rain detected)
      servoRunning = false;
      digitalWrite(RELAYPIN, LOW);  // Turn OFF the relay to disable servo power
      Serial.println("Servo halted due to high potentiometer value.");
    } else if (currentTime - lastServoRunTime >= servoRunDuration) {
      servoRunning = false; // Stop the servo after some time
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

  // Read light sensor value (photoresistor)
  float analogValue = analogRead(LIGHTPIN);
  float voltage = analogValue / 4096. * 5;  // ESP32 ADC resolution is 12-bit (0-4095)
  float resistance = 2000 * voltage / (1 - voltage / 5);  // Calculate resistance in ohms
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA)); // Calculate lux

  // Turn on LED if light level is low
  int lightThreshold = 100; // Threshold for low light
  if (lux < lightThreshold) {
    digitalWrite(LEDPIN, HIGH); // Turn on the LED
    Serial.println("Low light detected. LED turned ON.");
  } else {
    digitalWrite(LEDPIN, LOW); // Turn off the LED
    Serial.println("Sufficient light detected. LED turned OFF.");
  }
  
  // Motion detection with PIR sensor
  int pirValue = digitalRead(PIRPIN);  // Read PIR sensor state
  if (pirValue == HIGH) {              // If motion is detected
    tone(BUZZERPIN, 300);     // Turn on the buzzer
    if (pirState == LOW) {             
      Serial.println("Motion detected!");
      pirState = HIGH;
    }
    delay(1000);
    noTone(BUZZERPIN);
  } else {                             // If no motion is detected
    noTone(BUZZERPIN);      // Turn off the buzzer
    if (pirState == HIGH) {           
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
  Serial.print(", Lux: ");
  Serial.println(lux);

  delay(100);  
}
