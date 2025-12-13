#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// ---------------- WIFI SETTINGS ----------------
const char* ssid = "OLIN-DEVICES";
const char* password = "BestOval4Engineers!";

// ---------------- UDP SETTINGS -----------------
WiFiUDP udp;
const unsigned int localUdpPort = 54322;  // must match ROS UDP_PORT
char packetBuffer[255]; // Buffer to hold incoming packet

// Track last command sender for IMU data transmission
IPAddress lastSenderIP;
unsigned int lastSenderPort = 0;
bool hasValidSender = false;

// ---------------- IMU SETTINGS -----------------
MPU6050 accelgyro;
int16_t ax, ay, az; // Accelerometer (raw)
int16_t gx, gy, gz; // Gyroscope (raw)

// Scale factors (LSB/unit) based on the default settings of the library:
// Accel: MPU6050_ACCEL_FS_2 -> Sensitivity = 16384 LSB/g
const float ACCEL_SCALE_FACTOR = 16384.0;
// Gyro: MPU6050_GYRO_FS_250 -> Sensitivity = 131.0 LSB/deg/s
const float GYRO_SCALE_FACTOR = 131.0;

// IMU timing control
unsigned long lastImuReadTime = 0;
const unsigned long IMU_READ_INTERVAL = 10; // 10ms = 100Hz

// ---------------- MOTOR DEFINITIONS ----------------
#define MAX_SPEED 255

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *wheel1 = AFMS.getMotor(2);
Adafruit_DCMotor *wheel2 = AFMS.getMotor(3);
Adafruit_DCMotor *wheel3 = AFMS.getMotor(4);

// ---------------- FAN / ESC ------------------------
Servo esc;
int fanPulse = 1000;  // microseconds; expected range 1000–2000

// ---------------- MARKER SERVO ------------------------
Servo markerServo;
const int MARKER_UP_ANGLE = 0;   // Servo angle for marker up (False)
const int MARKER_DOWN_ANGLE = 160;  // Servo angle for marker down (True)

// ---------------- SAFETY STATE -----------------------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 50; // Reduced to 50ms for faster response
bool hasReceivedCommand = false;

// ---------------- DEBUG SETTINGS --------------------
#define DEBUG_MODE false  // Set to true to enable Serial prints (slows down processing)

// ---------------- HELPER: MOTOR SPEED SET -------------------
void setMotorSpeed(Adafruit_DCMotor *motor, float velocity) {
  if (velocity > 1.0) velocity = 1.0;
  if (velocity < -1.0) velocity = -1.0;

  int speed = int(fabs(velocity) * MAX_SPEED);

  if (velocity > 0) {
    motor->run(FORWARD);
  } else if (velocity < 0) {
    motor->run(BACKWARD);
  } else {
    motor->run(RELEASE);
  }

  motor->setSpeed(speed);

  #if DEBUG_MODE
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(" Dir: ");
    Serial.println(velocity > 0 ? "FWD" : (velocity < 0 ? "BWD" : "STOP"));
  #endif
}

// ---------------- HELPER: FAN SET -------------------
void setFanPulse(int pulse) {
  if (pulse < 1000) pulse = 1000;
  if (pulse > 2000) pulse = 2000;
  fanPulse = pulse;
  esc.writeMicroseconds(fanPulse);
  #if DEBUG_MODE
    Serial.print("Fan pulse set to: ");
    Serial.println(fanPulse);
  #endif
}

// ---------------- HELPER: MARKER SET ----------------
void setMarkerAngle(int angleDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  markerServo.write(angleDeg);
  #if DEBUG_MODE
    Serial.print("Marker angle set to: ");
    Serial.println(angleDeg);
  #endif
}

// ---------------- HELPER: UPDATE SENDER INFO ----------------
void updateSenderInfo() {
  lastSenderIP = udp.remoteIP();
  lastSenderPort = udp.remotePort();
  hasValidSender = true;
  #if DEBUG_MODE
    Serial.print("Updated sender: ");
    Serial.print(lastSenderIP);
    Serial.print(":");
    Serial.println(lastSenderPort);
  #endif
}

// ---------------- HELPER: READ AND SEND IMU DATA ----------------
void readAndSendImu() {
  // Read raw sensor data
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to engineering units
  float accelX_g = ax / ACCEL_SCALE_FACTOR;
  float accelY_g = ay / ACCEL_SCALE_FACTOR;
  float accelZ_g = az / ACCEL_SCALE_FACTOR;
  float gyroX_dps = gx / GYRO_SCALE_FACTOR;
  float gyroY_dps = gy / GYRO_SCALE_FACTOR;
  float gyroZ_dps = gz / GYRO_SCALE_FACTOR;

  // Send IMU data if we have a valid sender
  if (hasValidSender) {
    // Format: "IMU ax ay az gx gy gz" with 3 decimal places
    char imuBuffer[100];
    snprintf(imuBuffer, sizeof(imuBuffer), "IMU %.3f %.3f %.3f %.3f %.3f %.3f",
             accelX_g, accelY_g, accelZ_g, gyroX_dps, gyroY_dps, gyroZ_dps);

    udp.beginPacket(lastSenderIP, lastSenderPort);
    udp.write((uint8_t*)imuBuffer, strlen(imuBuffer));
    udp.endPacket();

    #if DEBUG_MODE
      Serial.print("Sent IMU: ");
      Serial.println(imuBuffer);
    #endif
  }
}

// ===================== SETUP ========================
void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize

  // 1. Setup Motors
  AFMS.begin();

  // 2. Setup ESC
  esc.attach(10);
  esc.writeMicroseconds(1000);

  // 3. Setup Marker Servo
  markerServo.attach(8);
  setMarkerAngle(MARKER_UP_ANGLE);  // Start with marker up

  // 4. Initialize IMU
  Wire.begin();
  Serial.println("Initializing MPU-6050...");
  accelgyro.initialize();

  // Verify IMU connection
  if (accelgyro.testConnection()) {
    Serial.println("MPU-6050 connection successful!");
  } else {
    Serial.println("MPU-6050 connection failed! Check wiring.");
    // Continue anyway - IMU will just fail silently
  }

  // 5. Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  // 6. Start UDP
  udp.begin(localUdpPort);
  Serial.print("Listening for UDP commands on port ");
  Serial.println(localUdpPort);

  // Arming delay for ESC
  delay(3000);
  Serial.println("Ready!");
  
  lastImuReadTime = millis();
}

// ====================== LOOP ========================
void loop() {
  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();

  if (packetSize > 0) {
    // Discard queued packets; keep only the latest
    while (udp.parsePacket() > 0) {
      udp.read(packetBuffer, 255);
    }

    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;

    // Update sender info for IMU transmission
    updateSenderInfo();

    if (strncmp(packetBuffer, "CMD", 3) == 0) {
      float v1 = 0.0, v2 = 0.0, v3 = 0.0;
      bool parseSuccess = false;

      char bufferCopy[255];
      strncpy(bufferCopy, packetBuffer, sizeof(bufferCopy));
      bufferCopy[sizeof(bufferCopy) - 1] = 0;

      char *token = strtok(bufferCopy, " ");
      if (token && strcmp(token, "CMD") == 0) {
        token = strtok(NULL, " ");
        if (token) {
          v1 = atof(token);
          token = strtok(NULL, " ");
          if (token) {
            v2 = atof(token);
            token = strtok(NULL, " \n\r");
            if (token) {
              v3 = atof(token);
              parseSuccess = true;
            }
          }
        }
      }

      if (parseSuccess) {
        lastCommandTime = millis();
        hasReceivedCommand = true;
        setMotorSpeed(wheel1, v1);
        setMotorSpeed(wheel2, v2);
        setMotorSpeed(wheel3, v3);
      }
    }
    else if (strncmp(packetBuffer, "FAN", 3) == 0) {
      int fanVal = fanPulse;
      bool parseSuccess = false;

      char bufferCopy[255];
      strncpy(bufferCopy, packetBuffer, sizeof(bufferCopy));
      bufferCopy[sizeof(bufferCopy) - 1] = 0;

      char *token = strtok(bufferCopy, " ");
      if (token && strcmp(token, "FAN") == 0) {
        token = strtok(NULL, " \n\r");
        if (token) {
          fanVal = atoi(token);
          parseSuccess = true;
        }
      }

      if (parseSuccess) {
        setFanPulse(fanVal);
      }
    }
    else if (strncmp(packetBuffer, "MARKER_ANGLE", 12) == 0) {
      // MARKER_ANGLE command expects: "MARKER_ANGLE <deg>"
      int angleVal = MARKER_UP_ANGLE; // default to up if parse fails
      bool parseSuccess = false;

      char bufferCopy[255];
      strncpy(bufferCopy, packetBuffer, sizeof(bufferCopy));
      bufferCopy[sizeof(bufferCopy) - 1] = 0;

      char *token = strtok(bufferCopy, " ");
      if (token && strcmp(token, "MARKER_ANGLE") == 0) {
        token = strtok(NULL, " \n\r");
        if (token) {
          angleVal = atoi(token);
          parseSuccess = true;
        }
      }

      if (parseSuccess) {
        setMarkerAngle(angleVal);
      }
    }
    else if (strncmp(packetBuffer, "MARKER", 6) == 0) {
      // MARKER command expects: "MARKER <0|1>" where 1 = down (True), 0 = up (False)
      int markerVal = 0;
      bool parseSuccess = false;

      char bufferCopy[255];
      strncpy(bufferCopy, packetBuffer, sizeof(bufferCopy));
      bufferCopy[sizeof(bufferCopy) - 1] = 0;

      char *token = strtok(bufferCopy, " ");
      if (token && strcmp(token, "MARKER") == 0) {
        token = strtok(NULL, " \n\r");
        if (token) {
          markerVal = atoi(token);
          parseSuccess = true;
        }
      }

      if (parseSuccess) {
        if (markerVal == 1) {
          setMarkerAngle(MARKER_DOWN_ANGLE);
        } else {
          setMarkerAngle(MARKER_UP_ANGLE);
        }
      }
    }
  }

  // Fast safety timeout check for motors
  if (hasReceivedCommand && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
    wheel1->run(RELEASE);
    wheel2->run(RELEASE);
    wheel3->run(RELEASE);
  }

  // Read and send IMU data at 100Hz (every 10ms)
  unsigned long currentTime = millis();
  if (currentTime - lastImuReadTime >= IMU_READ_INTERVAL) {
    readAndSendImu();
    lastImuReadTime = currentTime;
  }
}

