#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h> // Changed to Float32 for Yaw data
#include <Wire.h>
#include <MPU6050.h>

// ======================================================
// ================= GLOBAL VARIABLES ===================
// ======================================================

// ROS Objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_init_options_t init_options;

// MPU Objects
MPU6050 mpu;
float yaw = 0.0;
unsigned long lastTimeYaw = 0;

// Macros for Error Checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {while(1);} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

// ======================================================
// ================= HELPER FUNCTIONS ===================
// ======================================================

// Helper to keep angle between -180 and 180
float normalizeAngle(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// ======================================================
// ================= TIMER CALLBACK =====================
// ======================================================
// This function runs every 20ms (defined in setup)
// It reads the IMU, calculates Yaw, and publishes the message

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    
    // 1. Get Raw Data from MPU
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 2. Calculate Time passed (dt)
    unsigned long now = millis();
    float dt = (now - lastTimeYaw) / 1000.0;
    // Handle first run edge case to prevent huge dt
    if (lastTimeYaw == 0) dt = 0; 
    lastTimeYaw = now;

    // 3. Calculate Yaw
    // 131.0 is the sensitivity for default gyro range
    float gyroZ_degPerSec = gz / 131.0;
    yaw += gyroZ_degPerSec * dt;
    yaw = normalizeAngle(yaw);

    // 4. Publish to ROS
    msg.data = yaw;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    
    // Optional: Print to Serial for debugging (may slow down loop if too fast)
    // Serial.print("Publishing Yaw: ");
    // Serial.println(msg.data);
  }
}

// ======================================================
// ======================= SETUP ========================
// ======================================================

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // --- MPU SETUP ---
    // We do MPU setup first to ensure it's ready before ROS starts
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      while (1); 
    }
    
    Serial.println("Calibrating Gyro... Keep still!");
    mpu.CalibrateGyro();
    mpu.PrintActiveOffsets();
    
    // Reset timer for yaw calc
    lastTimeYaw = millis();

    // --- WIFI TRANSPORT ---
    set_microros_wifi_transports("OLIN-DEVICES", "BestOval4Engineers!", "192.168.1.200", 8888);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // --- INIT OPTIONS & DOMAIN ID ---
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 81)); // Your Domain ID

    // --- NODE & SUPPORT ---
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_imu_node", "", &support));

    // --- PUBLISHER INIT ---
    // Publishing to topic "imu_yaw" with Float32 type
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "imu_yaw"));

    // --- TIMER INIT ---
    // Create a timer to trigger the callback at 50Hz (20ms)
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // --- EXECUTOR ---
    // Note: We need 1 handle for the timer
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// ======================================================
// ======================== LOOP ========================
// ======================================================

void loop() {
    // Spin the executor to handle the timer callback
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}