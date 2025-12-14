/*
 * MERGED FIRMWARE: RP2040 Connect + TB6612FNG + UDP Control
 * HARDCODED IP VERSION: Uses targetIP only
 */

 #include <SPI.h>
 #include <WiFiNINA.h> 
 #include <WiFiUdp.h>
 #include <Wire.h>
 #include <Servo.h>
 #include "I2Cdev.h"
 #include "MPU6050.h"
 
 // ---------------- WIFI SETTINGS ----------------
 const char* ssid     = "OLIN-DEVICES";
 const char* password = "BestOval4Engineers!";
 
 // ---------------- UDP SETTINGS -----------------
 WiFiUDP udp;
 const unsigned int localUdpPort = 54322;
 char packetBuffer[255]; 
 
// Target Pi IP (update if it changes)
IPAddress targetIP(192, 168, 34, 201); // Current Pi Wi-Fi IP
 const unsigned int targetPort = 54322;
 
 // ---------------- HARDWARE PINOUT ----------------
 // Motor #1 (Front)
 #define M1_BIN1 A2
 #define M1_BIN2 A3
 #define M1_PWM  4   // D4
 
 // Motor #2 (Left Rear)
 #define M2_PWMA 3   // D3
 #define M2_AIN2 7   // D7
 #define M2_AIN1 8   // D8
 
 // Motor #3 (Right Rear)
 #define M3_BIN1 A1
 #define M3_BIN2 A0
 #define M3_PWM  5   // D5
 
 // ESC (Brushless Motor / Fan)
 #define ESC_PIN 6   // D6
 
 // ---------------- IMU SETTINGS -----------------
 MPU6050 accelgyro;
 int16_t ax, ay, az;
 int16_t gx, gy, gz;
 
 const float ACCEL_SCALE_FACTOR = 16384.0;
 const float GYRO_SCALE_FACTOR = 131.0;
 
 unsigned long lastImuReadTime = 0;
 const unsigned long IMU_READ_INTERVAL = 10; // 10ms = 100Hz
 
 // ---------------- OBJECTS ----------------
 Servo esc;
 int fanPulse = 1000; 
 
 // ---------------- SAFETY STATE ----------------
 unsigned long lastCommandTime = 0;
 const unsigned long COMMAND_TIMEOUT = 50; 
 bool hasReceivedCommand = false;
 
 // ---------------- HELPER: SET TB6612FNG MOTOR ----------------
 void setMotorSpeed(int motorID, float velocity) {
   if (velocity > 1.0) velocity = 1.0;
   if (velocity < -1.0) velocity = -1.0;
 
   int pwmVal = int(fabs(velocity) * 255);
   int pinPWM, pinDir1, pinDir2;
 
   if (motorID == 1) {
     pinPWM = M1_PWM; pinDir1 = M1_BIN1; pinDir2 = M1_BIN2;
   } else if (motorID == 2) {
     pinPWM = M2_PWMA; pinDir1 = M2_AIN1; pinDir2 = M2_AIN2;
   } else if (motorID == 3) {
     pinPWM = M3_PWM; pinDir1 = M3_BIN1; pinDir2 = M3_BIN2;
   } else return; 
 
   if (velocity > 0) {
     digitalWrite(pinDir1, HIGH); digitalWrite(pinDir2, LOW);
     analogWrite(pinPWM, pwmVal);
   } else if (velocity < 0) {
     digitalWrite(pinDir1, LOW); digitalWrite(pinDir2, HIGH);
     analogWrite(pinPWM, pwmVal);
   } else {
     digitalWrite(pinDir1, LOW); digitalWrite(pinDir2, LOW);
     analogWrite(pinPWM, 0);
   }
 }
 
 // ---------------- HELPER: FAN CONTROL ----------------
 void setFanPulse(int pulse) {
   if (pulse < 1000) pulse = 1000;
   if (pulse > 2000) pulse = 2000;
   fanPulse = pulse;
   esc.writeMicroseconds(fanPulse);
 }
 
 // ---------------- HELPER: IMU SENDER ----------------
 void readAndSendImu() {
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
   float accelX_g = ax / ACCEL_SCALE_FACTOR;
   float accelY_g = ay / ACCEL_SCALE_FACTOR;
   float accelZ_g = az / ACCEL_SCALE_FACTOR;
   float gyroX_dps = gx / GYRO_SCALE_FACTOR;
   float gyroY_dps = gy / GYRO_SCALE_FACTOR;
   float gyroZ_dps = gz / GYRO_SCALE_FACTOR;
 
   // Always send to manually entered Pi IP
   char imuBuffer[100];
   snprintf(imuBuffer, sizeof(imuBuffer), "IMU %.3f %.3f %.3f %.3f %.3f %.3f",
            accelX_g, accelY_g, accelZ_g, gyroX_dps, gyroY_dps, gyroZ_dps);
   
   udp.beginPacket(targetIP, targetPort);
   udp.write((uint8_t*)imuBuffer, strlen(imuBuffer));
   udp.endPacket();
 
   // Uncomment below to debug sending logic
   // Serial.print("Sent IMU to Pi ");
   // Serial.print(targetIP);
   // Serial.print(":");
   // Serial.println(targetPort);
 }
 
 
 // ===================== SETUP ========================
 void setup() {
   Serial.begin(115200);
   
   pinMode(M1_BIN1, OUTPUT); pinMode(M1_BIN2, OUTPUT); pinMode(M1_PWM, OUTPUT);
   pinMode(M2_AIN1, OUTPUT); pinMode(M2_AIN2, OUTPUT); pinMode(M2_PWMA, OUTPUT);
   pinMode(M3_BIN1, OUTPUT); pinMode(M3_BIN2, OUTPUT); pinMode(M3_PWM, OUTPUT);
 
   esc.attach(ESC_PIN);
   esc.writeMicroseconds(1000); 
   
   Wire.begin();
   Serial.println("Initializing MPU-6050...");
   accelgyro.initialize();
   if (accelgyro.testConnection()) {
     Serial.println("MPU-6050 connection successful!");
   } else {
     Serial.println("MPU-6050 connection failed! Check wiring.");
   }
 
   if (WiFi.status() == WL_NO_MODULE) {
     Serial.println("Communication with WiFi module failed!");
     while (true);
   }
 
   Serial.print("Connecting to Wi-Fi: ");
   Serial.println(ssid);
   
   WiFi.begin(ssid, password);
 
   // --- WAIT FOR VALID IP ---
   while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0,0,0,0)) {
     Serial.print(".");
     delay(1000);
     Serial.print(" Status: "); Serial.print(WiFi.status());
     Serial.print(" IP: "); Serial.println(WiFi.localIP());
   }
   Serial.println("\nConnected!");
   Serial.print("IP Address: ");
   Serial.println(WiFi.localIP());
 
   udp.begin(localUdpPort);
   Serial.print("Listening on UDP port: ");
   Serial.println(localUdpPort);
 
   delay(2000); 
   Serial.println("System Ready.");
 }
 
 // ====================== LOOP ========================
 void loop() {
   // 1. PARSE INCOMING UDP
   int packetSize = udp.parsePacket();
   if (packetSize > 0) {
     
     Serial.print("Received packet size: ");
     Serial.println(packetSize);
 
     int len = udp.read(packetBuffer, 255);
     if (len > 0) packetBuffer[len] = 0;
 
     Serial.print("Payload: ");
     Serial.println(packetBuffer);
 
     // CMD: "CMD v1 v2 v3" 
     if (strncmp(packetBuffer, "CMD", 3) == 0) {
       float v1=0, v2=0, v3=0;
       char* token = strtok(packetBuffer, " "); 
       token = strtok(NULL, " "); if(token) v1 = atof(token);
       token = strtok(NULL, " "); if(token) v2 = atof(token);
       token = strtok(NULL, " "); if(token) v3 = atof(token);
 
       lastCommandTime = millis();
       hasReceivedCommand = true;
       
       setMotorSpeed(1, v1);
       setMotorSpeed(2, v2);
       setMotorSpeed(3, v3);
     }
     // FAN: "FAN pulse"
     else if (strncmp(packetBuffer, "FAN", 3) == 0) {
       int val = 1000;
       char* token = strtok(packetBuffer, " "); 
       token = strtok(NULL, " ");
       if(token) val = atoi(token);
       setFanPulse(val);
       Serial.print("Fan set to: ");
       Serial.println(val);
     }
   }
 
   // 2. SAFETY TIMEOUT
   if (hasReceivedCommand && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
     setMotorSpeed(1, 0);
     setMotorSpeed(2, 0);
     setMotorSpeed(3, 0);
     hasReceivedCommand = false; 
   }
 
   // 3. IMU TELEMETRY
   if (millis() - lastImuReadTime >= IMU_READ_INTERVAL) {
     readAndSendImu();
     lastImuReadTime = millis();
   }
 }