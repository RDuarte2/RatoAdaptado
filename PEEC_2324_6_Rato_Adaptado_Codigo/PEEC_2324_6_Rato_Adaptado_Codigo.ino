#include <Wire.h>
#include "MPU6050.h"
#include "BleMouse.h"
#include <Adafruit_LPS35HW.h>

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
MPU6050 mpu;
BleMouse bleMouse;

// IMU constants
#define THRESHOLD 0
#define SENSIBILIDADE 20

// Pressure constants
#define LONG_PRESS_TIME 1000  // 1000 milliseconds
#define SHORT_PRESS_TIME 500  // 500 milliseconds
#define THRESHOLD_PUFF 5
#define THRESHOLD_SIP 5

// Pin Definitions
#define BTLED_PIN 3
#define BATLED_PIN 17
#define MEDBAT_PIN 2
#define CALIBRATION_PIN 0

// Timers
float timeStep = 0.02;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float old_yaw = 0;
float old_pitch = 0;
float old_roll = 0;
float yaw_rate = 0;
float pitch_rate = 0;
float roll_rate = 0;

// Pressure variables
int lastState_Sip = LOW;  // the previous state from the input pin
int currentState_Sip;     // the current reading from the input pin
int lastState_Puff = LOW;
int currentState_Puff;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;

// Timer
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned int time_bt = 1000;
unsigned int time_bat = 500;

void Mouse_Click(void);
void Mouse_Movement(void);
void Battery(void);

void setup() {
  Serial.begin(115200);

  pinMode(BTLED_PIN, OUTPUT);
  pinMode(MEDBAT_PIN, INPUT);
  pinMode(BATLED_PIN, OUTPUT);
  pinMode(CALIBRATION_PIN, INPUT_PULLUP);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Check if sensor is found
  if (!lps35hw.begin_I2C()) {
    Serial.println("Couldn't find LPS35HW chip");
    while (1)
      ;
  }
  Serial.println("Found LPS35HW chip");

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(THRESHOLD);

  // Set zero as reference
  lps35hw.zeroPressure();

  // Start mouse
  Serial.println("Starting BLE Mouse!");
  bleMouse.begin();
}

void loop() {
  currentTime = millis();

  Mouse_Movement();
  Mouse_Click();
  Battery();

  if (digitalRead(CALIBRATION_PIN) == LOW) {
    mpu.calibrateGyro();
    Serial.println("calibrated");
  }

  if (bleMouse.isConnected()) {
    digitalWrite(BTLED_PIN, HIGH);
    // if (currentTime - previousTime >= time_bt) {
    //   Serial.println("Connected!");
    //   previousTime = currentTime;
    // }
  } else {
    digitalWrite(BTLED_PIN, LOW);
    // if (currentTime - previousTime >= time_bt) {
    //   Serial.println("Disconnected!");
    //   previousTime = currentTime;
    // }
  }

  // Wait to full timeStep period
  delay(10);
}

void Mouse_Click(void) {
  if (lps35hw.readPressure() > THRESHOLD_PUFF) {
    currentState_Puff = HIGH;

  } else if (lps35hw.readPressure() < -THRESHOLD_SIP) {
    currentState_Sip = HIGH;

  } else {
    currentState_Puff = LOW;
    currentState_Sip = LOW;
  }

  if (lastState_Sip == LOW && currentState_Sip == HIGH) {  // button is pressed
    pressedTime = millis();
    isPressing = true;

  } else if (lastState_Sip == HIGH && currentState_Sip == LOW) {  // button is released
    releasedTime = millis();
    isPressing = false;
    isLongDetected = false;
    bleMouse.release(MOUSE_LEFT);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration < SHORT_PRESS_TIME) {
      Serial.println("A short press is detected");
      bleMouse.click(MOUSE_RIGHT);
      Serial.println("Right Click");
    }
  }

  if (lastState_Puff == LOW && currentState_Puff == HIGH) {  // button is pressed
    pressedTime = millis();
    isPressing = true;

  } else if (lastState_Puff == HIGH && currentState_Puff == LOW) {  // button is released
    releasedTime = millis();
    isPressing = false;
    isLongDetected = false;
    bleMouse.release(MOUSE_LEFT);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration < SHORT_PRESS_TIME) {
      Serial.println("A short press is detected");
      bleMouse.click(MOUSE_LEFT);
      Serial.println("Left Click");
    }
  }

  if (isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if (pressDuration > LONG_PRESS_TIME) {
      Serial.println("A long press is detected");
      bleMouse.press(MOUSE_LEFT);
      isLongDetected = true;
    }
  }

  // save the the last state
  lastState_Puff = currentState_Puff;
  lastState_Sip = currentState_Sip;
}

void Mouse_Movement(void) {
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Calculate degree variations
  yaw_rate = (yaw - old_yaw) * SENSIBILIDADE;
  pitch_rate = (pitch - old_pitch) * SENSIBILIDADE;
  roll_rate = (roll - old_roll) * SENSIBILIDADE;

  old_yaw = yaw;
  old_pitch = pitch;
  old_roll = roll;

  // Serial.print(" Pitch = ");
  // Serial.print(pitch);
  // Serial.print(" Roll = ");
  // Serial.print(roll);
  // Serial.print(" Yaw = ");
  // Serial.println(yaw);

  // Serial.print(" Yaw Rate = ");
  // Serial.println(yaw_rate);
  // Serial.print(" Pitch Rate = ");
  // Serial.println(pitch_rate);
  // Serial.print(" Roll Rate = ");
  // Serial.println(roll_rate);

  // bleMouse.move(pitch_rate, yaw_rate, 0);
  bleMouse.move(pitch_rate, -roll_rate, 0);
}

void Battery(void) {
  float v_ADC;
  float bateria;
  int bateria_perc;

  v_ADC = analogRead(2) * 3.3 / 4096;  // Tensão do ADC
  // Serial.print("ADC: ");
  // Serial.println(v_ADC);
  bateria = (127 * v_ADC) / 100;
  // Serial.print("Bateria: ");
  // Serial.println(bateria);

  bateria_perc = map(bateria, 0, 4.2, 0, 100);

  Serial.print("Bateria %: ");
  Serial.println(bateria_perc);

  // if (bateria_perc < 20) {
  //   digitalWrite(BATLED_PIN, HIGH);
  // } else if (bateria_perc < 10) {
  //   digitalWrite(BATLED_PIN, HIGH);
  //   if (currentTime - previousTime >= time_bat) {
  //     digitalWrite(BATLED_PIN, LOW);
  //     previousTime = currentTime;
  //   }
  // } else {
  //   digitalWrite(BATLED_PIN, LOW);
  // }

  //delay(100);
}