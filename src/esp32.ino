#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ESP32Servo.h>   
#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

#define DHTPIN 4            
#define DHTTYPE DHT22       
#define POTPIN 35           
#define PIRPIN 32           
#define BUZZERPIN 23        
#define SERVOPIN 25         
#define RELAYPIN 22         
#define LIGHTPIN 34         
#define LEDPIN 27           
#define SWITCHPIN 33

const char* ssid = "Wokwi-GUEST";
const char* password = "";

//***Set server***
const char* mqttServer = "broker.hivemq.com"; 
int port = 1883;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);  
Servo wateringServo;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
AccelStepper fanStepper(AccelStepper::FULL4WIRE, 3, 2, 13, 14);

bool IOTsystem = false;
bool watersystem = false;
bool setupenv = false;

int pirState = LOW;
unsigned long lastServoRunTime = 0;      // Stores the last time the servo was activated
const unsigned long servoInterval = 3000;  // Every 3s servo runs
const unsigned long servoRunDuration = 30000;  // 30s, Running duration of servo
bool servoRunning = false;
bool servoDirectionUp = true;  

const float GAMMA = 0.7;       // Gamma value for the photoresistor
const float RL10 = 50;         // Resistance of LDR

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
      mqttClient.subscribe("/wateringTKMQ/system", 1);
      mqttClient.subscribe("/wateringTKMQ/autowatering", 1);
      mqttClient.subscribe("/wateringTKMQ/env", 1);
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
  if (String(topic) == "/wateringTKMQ/temp" || String(topic) == "/wateringTKMQ/humid") {
    return; 
  }

  Serial.print("Received message on topic: ");
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);

  //***Code here to process the received package***
  if (String(topic) == "/wateringTKMQ/system") {
    if (strMsg == "OFF"){
      IOTsystem = false;
      Serial.println("System turned OFF via MQTT.");
    }
    else {
      IOTsystem = true;
      Serial.println("System turned ON via MQTT.");
    } 
  }
  else if (String(topic) == "/wateringTKMQ/autowatering") {
    if (strMsg == "OFF") {
      watersystem = false;
    }
    else {
      watersystem = true;
    }
  }
  else if (String(topic) == "/wateringTKMQ/env") {
    if (strMsg == "OFF") {
      setupenv = false;
    }
    else {
      setupenv = true;
    }
  }
}

void setup() {
  fanStepper.setMaxSpeed(8000);
  Serial.begin(115200);
  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(90);

  lcd.init();
  lcd.backlight();

  dht.begin();

  wateringServo.attach(SERVOPIN);

  pinMode(PIRPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(SWITCHPIN, INPUT_PULLUP);

  digitalWrite(RELAYPIN, LOW);
  digitalWrite(BUZZERPIN, LOW);
  digitalWrite(LEDPIN, HIGH);
}

void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  if (!IOTsystem) {
    return;
  }
  // Read temperature and humidity
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Publish temperature and humidity to MQTT
  char tempBuffer[50];
  char humidBuffer[50];

  // Convert temperature and humidity to string
  snprintf(tempBuffer, sizeof(tempBuffer), "%.2f", temperature);
  snprintf(humidBuffer, sizeof(humidBuffer), "%.2f", humidity);

  // Publish topics
  mqttClient.publish("/wateringTKMQ/temp", tempBuffer);
  mqttClient.publish("/wateringTKMQ/humid", humidBuffer);

  // Display temperature, humidity
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(temperature) + " C");
  lcd.setCursor(0, 1);
  lcd.print("Humid: " + String(humidity) + " %");

  // Enable fan
  bool manualOverride = digitalRead(SWITCHPIN);
  if ((temperature > 30.0 || manualOverride) && setupenv) {
    fanStepper.setSpeed(3000);
    fanStepper.runSpeed();
    Serial.println("Fan is on");
  }
  else {
    fanStepper.stop();
  }

  int potValue = analogRead(POTPIN);
  unsigned long currentTime = millis();
  if (currentTime - lastServoRunTime >= servoInterval && !servoRunning && watersystem) {
    servoRunning = true;
    lastServoRunTime = currentTime; 
    digitalWrite(RELAYPIN, HIGH);   
  }

  if (servoRunning && watersystem) {
    if (potValue > 350) {  // rain detected
      servoRunning = false;
      digitalWrite(RELAYPIN, LOW); 
      Serial.println("Servo halted due to high potentiometer value.");
    } else if (currentTime - lastServoRunTime >= servoRunDuration) {
      servoRunning = false; 
      digitalWrite(RELAYPIN, LOW);  
    }  else {
        static int currentAngle = 0; 
        if (servoDirectionUp) {
            currentAngle += 10; 
            if (currentAngle >= 180) {
                currentAngle = 180;
                servoDirectionUp = false; 
            }
        } else {
            currentAngle -= 10; 
            if (currentAngle <= 0) {
                currentAngle = 0;
                servoDirectionUp = true; 
            }
        }
        wateringServo.write(currentAngle); 
        delay(10); 
    }
  } else {
    wateringServo.write(0); 
    digitalWrite(RELAYPIN, LOW); 
  }

  // Read light sensor value (photoresistor)
  float analogValue = analogRead(LIGHTPIN);
  float voltage = analogValue / 4096. * 5;  // ESP32 ADC resolution is 12-bit (0-4095)
  float resistance = 2000 * voltage / (1 - voltage / 5);  // Calculate resistance in ohms
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA)); // Calculate lux
  
  // Turn on LED if light level is low
  int lightThreshold = 100; // 
  if ((lux < lightThreshold) && setupenv) {
    digitalWrite(LEDPIN, HIGH);
    Serial.println("Low light detected. LED turned ON.");
  } else {
    digitalWrite(LEDPIN, LOW); 
  }
  
  // Motion detection
  int pirValue = digitalRead(PIRPIN);  
  if (pirValue == HIGH) {              
    tone(BUZZERPIN, 300);     
    if (pirState == LOW) {             
      Serial.println("Motion detected!");
      pirState = HIGH;
    }
    delay(1000);
    noTone(BUZZERPIN);
  } else {                         
    noTone(BUZZERPIN);      
    if (pirState == HIGH) {           
      Serial.println("Motion ended!");
      pirState = LOW;
    }
  }

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
