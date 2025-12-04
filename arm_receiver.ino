#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Servo pins
#define BASE_SERVO_PIN      13
#define SHOULDER_SERVO_PIN  12
#define ELBOW_SERVO_PIN     14
#define WRIST_SERVO_PIN     27
#define GRIPPER_SERVO_PIN   26

// Step movement parameters
const int STEP_SIZE = 2;                // degrees per step
const unsigned long STEP_INTERVAL = 20; // ms between steps (~50 Hz)

// Flex sensor calibration (example values – adjust after testing)
const int FLEX_OPEN_RAW    = 1500;  // flex reading when hand is open
const int FLEX_CLOSED_RAW  = 3000;  // flex reading when hand is closed
const int GRIPPER_OPEN_ANGLE   = 130; // servo angle for open
const int GRIPPER_CLOSED_ANGLE = 20;  // servo angle for closed

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

// Data structure received from glove
typedef struct {
  int pitch;
  int roll;
  int flex;
} Message;

Message incomingData;

// Target angles set by ESP-NOW callback
volatile int targetBaseAngle      = 90;
volatile int targetShoulderAngle  = 90;
volatile int targetElbowAngle     = 90;
volatile int targetWristAngle     = 90;
volatile int targetGripperAngle   = 90;

// Current angles used to drive servos (step-wise)
int currentBaseAngle      = 90;
int currentShoulderAngle  = 90;
int currentElbowAngle     = 90;
int currentWristAngle     = 90;
int currentGripperAngle   = 90;

unsigned long lastStepTime = 0;

// Step one joint towards target
int stepTowards(int current, int target) {
  if (current == target) return current;

  int diff = target - current;
  if (abs(diff) <= STEP_SIZE) {
    return target;
  } else if (diff > 0) {
    return current + STEP_SIZE;
  } else {
    return current - STEP_SIZE;
  }
}

// ESP-NOW receive callback
void onReceive(const uint8_t *mac, const uint8_t *incomingDataPtr, int len) {
  memcpy((void *)&incomingData, incomingDataPtr, sizeof(incomingData));

  int pitch   = incomingData.pitch; // -60 to 60 (from transmitter)
  int roll    = incomingData.roll;  // -60 to 60
  int flexRaw = incomingData.flex;

  // Map roll & pitch to servo mechanical ranges (tune for your arm)
  int baseAngle     = map(roll,  -60, 60,  30, 150);  // base
  int shoulderAngle = map(pitch, -60, 60,  40, 140);  // shoulder
  int elbowAngle    = map(pitch, -60, 60,  30, 150);  // elbow
  int wristAngle    = map(roll,  -60, 60,  30, 150);  // wrist

  baseAngle     = constrain(baseAngle,     0, 180);
  shoulderAngle = constrain(shoulderAngle, 0, 180);
  elbowAngle    = constrain(elbowAngle,    0, 180);
  wristAngle    = constrain(wristAngle,    0, 180);

  // Map flex raw value to gripper angle
  int gripperAngle = map(flexRaw,
                         FLEX_OPEN_RAW, FLEX_CLOSED_RAW,
                         GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE);

  gripperAngle = constrain(gripperAngle,
                           min(GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE),
                           max(GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE));

  // Update targets
  targetBaseAngle     = baseAngle;
  targetShoulderAngle = shoulderAngle;
  targetElbowAngle    = elbowAngle;
  targetWristAngle    = wristAngle;
  targetGripperAngle  = gripperAngle;

  Serial.print("RX Pitch: ");
  Serial.print(pitch);
  Serial.print(" Roll: ");
  Serial.print(roll);
  Serial.print(" Flex: ");
  Serial.print(flexRaw);
  Serial.print(" | Targets -> B:");
  Serial.print(baseAngle);
  Serial.print(" S:");
  Serial.print(shoulderAngle);
  Serial.print(" E:");
  Serial.print(elbowAngle);
  Serial.print(" W:");
  Serial.print(wristAngle);
  Serial.print(" G:");
  Serial.println(gripperAngle);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed.");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  // Attach servos with pulse width limits (tune if needed)
  baseServo.attach(BASE_SERVO_PIN,         500, 2400);
  shoulderServo.attach(SHOULDER_SERVO_PIN, 500, 2400);
  elbowServo.attach(ELBOW_SERVO_PIN,       500, 2400);
  wristServo.attach(WRIST_SERVO_PIN,       500, 2400);
  gripperServo.attach(GRIPPER_SERVO_PIN,   500, 2400);

  // Start from neutral position
  baseServo.write(currentBaseAngle);
  shoulderServo.write(currentShoulderAngle);
  elbowServo.write(currentElbowAngle);
  wristServo.write(currentWristAngle);
  gripperServo.write(currentGripperAngle);

  Serial.println("Arm receiver setup complete.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastStepTime >= STEP_INTERVAL) {
    lastStepTime = now;

    // Step each joint towards its target
    currentBaseAngle     = stepTowards(currentBaseAngle,     targetBaseAngle);
    currentShoulderAngle = stepTowards(currentShoulderAngle, targetShoulderAngle);
    currentElbowAngle    = stepTowards(currentElbowAngle,    targetElbowAngle);
    currentWristAngle    = stepTowards(currentWristAngle,    targetWristAngle);
    currentGripperAngle  = stepTowards(currentGripperAngle,  targetGripperAngle);

    // Write updated positions to servos
    baseServo.write(currentBaseAngle);
    shoulderServo.write(currentShoulderAngle);
    elbowServo.write(currentElbowAngle);
    wristServo.write(currentWristAngle);
    gripperServo.write(currentGripperAngle);
  }
}
