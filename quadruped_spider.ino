#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN   110
#define SERVOMAX   490
#define SERVO_FREQ 50

const int HIP[4]  = {0, 2, 4, 6};
const int KNEE[4] = {1, 3, 5, 7};

bool crawling = false;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(1000);

  calibrateServos();
  Serial.println("Type commands: calibrate, dance, crawl, stop");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "calibrate") {
      crawling = false;
      calibrateServos();
    } else if (command == "dance") {
      crawling = false;
      dance(12);  // dance 3 times
    } else if (command == "crawl") {
      crawling = true;
    } else if (command == "stop") {
      crawling = false;
      Serial.println("Crawling stopped.");
    }
  }

  if (crawling) {
    crawlForward();
  }
}

void calibrateServos() {
  Serial.println("Calibrating hips and knees to 90 degrees...");
  for (int i = 0; i < 4; i++) {
    setAngle(HIP[i], 90);
    setAngle(KNEE[i], 90);
  }
  delay(1500);
}

void setAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void smoothMove(uint8_t channel, float fromAngle, float toAngle, int steps, int delayTime) {
  for (int i = 0; i <= steps; i++) {
    float angle = fromAngle + (toAngle - fromAngle) * i / steps;
    setAngle(channel, angle);
    delay(delayTime);
  }
}

void dance(int cycles) {
  int steps = 40;
  int delayTime = 15;
  float swayAmplitudeHip = 20;
  float swayAmplitudeKnee = 10;

  for (int c = 0; c < cycles; c++) {
    for (int step = 0; step <= steps; step++) {
      float t = (float)step / steps * 2 * PI;

      for (int i = 0; i < 4; i++) {
        float hipAngle = 90 + swayAmplitudeHip * sin(t);
        float kneeAngle = 90 + swayAmplitudeKnee * sin(t + PI / 2);
        setAngle(HIP[i], hipAngle);
        setAngle(KNEE[i], kneeAngle);
      }

      delay(delayTime);
    }
  }

  // Return to neutral pose
  for (int i = 0; i < 4; i++) {
    setAngle(HIP[i], 90);
    setAngle(KNEE[i], 90);
  }
  delay(500);
}

void crawlForward() {
  static bool phase = false;
  int liftAngle = 60;
  int standAngle = 90;
  int stepSize = 60;
  int steps = 15;
  int stepDelay = 10;

  if (!phase) {
    smoothMove(KNEE[0], standAngle, liftAngle, steps, stepDelay);
    smoothMove(KNEE[3], standAngle, liftAngle, steps, stepDelay);
    smoothMove(HIP[0], 90, 90 - stepSize, steps, stepDelay);
    smoothMove(HIP[3], 90, 90 - stepSize, steps, stepDelay);
    smoothMove(KNEE[0], liftAngle, standAngle, steps, stepDelay);
    smoothMove(KNEE[3], liftAngle, standAngle, steps, stepDelay);
    smoothMove(HIP[0], 90 - stepSize, 90, steps, stepDelay);
    smoothMove(HIP[3], 90 - stepSize, 90, steps, stepDelay);
  } else {
    smoothMove(KNEE[1], standAngle, liftAngle, steps, stepDelay);
    smoothMove(KNEE[2], standAngle, liftAngle, steps, stepDelay);
    smoothMove(HIP[1], 90, 90 + stepSize, steps, stepDelay);
    smoothMove(HIP[2], 90, 90 + stepSize, steps, stepDelay);
    smoothMove(KNEE[1], liftAngle, standAngle, steps, stepDelay);
    smoothMove(KNEE[2], liftAngle, standAngle, steps, stepDelay);
    smoothMove(HIP[1], 90 + stepSize, 90, steps, stepDelay);
    smoothMove(HIP[2], 90 + stepSize, 90, steps, stepDelay);
  }

  phase = !phase;
}
