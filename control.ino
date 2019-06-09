#include <Math.h>
#include <Servo.h>
#include <AceRoutine.h>
using namespace ace_routine;

Servo tilt;
Servo pan;
Servo rot1;
Servo rot2;

Servo servos[4];

//const int MIN_PWM = 553;
//const int MAX_PWM = 2425;
//const int MIN_PWM = 600;
//const int MAX_PWM = 2400;

int sensors[] = { A0, A1, A2, A3 };
unsigned int sensor_vals[4];
float current[4][10];
const int num_samples = 20;
const float V_ref = 2500;
const float sensitivity = 1000.0 / 400.0; // 1A per 400mV

const float P = 1.0;
const float D = 4.0;
float target[4];
float error[4];
float prev_error[4];

const float LEN_0 = 128.98; // pivot 0 to pivot 1
const float LEN_1 = 83.83; // pivot 1 to pivot 2
const float LEN_2 = 69.97; // pivot 2 to middle of l-bracket (43.22 + 26.75)

int iters = 0;

float compute_current(int sensor_val) {
  float voltage = 4.88 * sensor_val;
  return (voltage - V_ref) * sensitivity;
}

float rad2deg(float rad) {
  return 180.0 * rad / M_PI;
}

float deg2rad(float deg) {
  return M_PI * deg / 180.0;
}

void print_currents() {
  float total_current = 0.0;
  for (int k = 0; k < 4; k++) {
    total_current += current[k][0];
    Serial.print(current[k][0]);
    Serial.print(",");
  }
  Serial.print(total_current);
  Serial.print("\n");
}

void move_servo0(float deg) {
  // 40 tooth driving a 48 tooth gear -> 1.2 gear ratio
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
  //  0.00 deg ->  652 us
  // 90.00 deg -> 1740 us
  servos[0].writeMicroseconds(round(652 + (deg * 1.2)/0.0992647));
}

void move_servo1(float deg) {
  // 40 tooth driving a 48 tooth gear -> 1.2 gear ratio
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
  //  90.00 deg -> 1000 us
  // 180.00 deg -> 2114 us
  servos[1].writeMicroseconds(round(-114 + (deg * 1.2)/0.0969469));
}

void move_servo2(float deg) {
  // Direct connection
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
  //  90.00 deg -> 1460 us
  servos[2].writeMicroseconds(round(578 + (180.0-deg) / 0.102));
}

// TODO: easing and make servos move to target position over time
void move_to(float x, float y, float tx, float ty, float tilt) {
  // x-extent:
  // y-extent: 100mm - 192mm
  float offset_x = LEN_2; // x-offset due to pitcher
  float offset_y = 0.0;

  float target_x = x + offset_x;
  float target_y = y + offset_y;

  float dist2 = target_x*target_x + target_y*target_y;
  float gamma = atan2(target_y, target_x);
  float beta = acos((dist2 - LEN_0*LEN_0 - LEN_1*LEN_1)/(-2*LEN_0*LEN_1));
  float psi = M_PI - beta;
  float alpha = asin((LEN_1 * sin(psi)) / sqrt(dist2));

  float deg0 = rad2deg(gamma - alpha);
  float deg1 = 90.0 + rad2deg(psi);
  
  move_servo0(deg0);
  move_servo1(deg1);
  move_servo2(360.0 - (deg0 + deg1));
  servos[3].writeMicroseconds(1472);
}

// debug position
void debug_pos0(bool absolute) {
  // 0.00 deg, 90.00 deg, 90.00 deg, down
  if (absolute) {
    servos[0].writeMicroseconds(652);
    servos[1].writeMicroseconds(2114);
    servos[2].writeMicroseconds(1460);
    servos[3].writeMicroseconds(553);
  } else {
    move_servo0(0);
    move_servo1(180);
    servos[2].writeMicroseconds(1460);
    servos[3].writeMicroseconds(553);
  }
}

void debug_pos1(bool absolute) {
  // 90.00 deg, 90.00 deg, 90.00 deg, level
  if (absolute) {
    servos[0].writeMicroseconds(1740);
    servos[1].writeMicroseconds(1000);
//    servos[2].writeMicroseconds(); // TODO
  }
  else {
    move_servo0(90);
    move_servo1(90);
  }
}

// closest cup position
void pos0() {
  // 16.00 deg, x.xx deg, x.xx deg, level
  float deg = 16.00;
  float deg2 = -deg + 90 + rad2deg(acos((LEN_2 - LEN_0 * cos(deg2rad(deg)))/LEN_1));
  move_servo0(deg);
  move_servo1(deg2);
  move_servo2(360.0 - (deg + deg2));
  servos[3].writeMicroseconds(1472);
}

// furthest cup position
void pos1() {
  // 57.15 deg, 90.00 deg, 180.00 deg, level
  float deg = 57.15;
  move_servo0(57.15);
  move_servo1(122.85);
  move_servo2(180.0);
  servos[3].writeMicroseconds(1472);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servos[0].attach(3); // rot0
  servos[1].attach(5); // rot1
  servos[2].attach(6); // pan
  servos[3].attach(9); // tilt
//  servos[0].attach(3, MIN_PWM, MAX_PWM); // rot0
//  servos[1].attach(5, MIN_PWM, MAX_PWM); // rot1
//  servos[2].attach(6, MIN_PWM, MAX_PWM); // pan
//  servos[3].attach(9, MIN_PWM, MAX_PWM); // tilt

  move_to(0, 100, 0, 1, 90);
  
  CoroutineScheduler::setup();
}

COROUTINE(measure_current) {
  COROUTINE_LOOP() {
    // Take current samples
    static int sample = 0;
    for (sample = 0; sample < num_samples; sample++) {
      for (int k = 0; k < 4; k++) {
        sensor_vals[k] += analogRead(sensors[k]);
      }
      COROUTINE_DELAY(2);
    }
  
    // Shift over current readings
    for (int i = 1; i < 10; i++) {
      for (int k = 0; k < 4; k++) {
        current[k][i] = current[k][i - 1];
      }
    }
  
    // Compute new current and target
    for (int k = 0; k < 4; k++) {
      current[k][0] = compute_current(sensor_vals[k] / num_samples);
      //    for (int i = 0; i < 10; i++) {
      //      target[k] += current[k][i];
      //    }
      //    target[k] /= 10.0;
      target[k] = 200.0;
    }
  
    // Move the servos for compliance
    for (int k = 0; k < 4; k++) {
      error[k] = target[k] - current[k][0];
      float react = P * error[k] + D * (error[k] - prev_error[k]);
  
      //    if (react > 0) {
      //      servos[k].write(servos[k].read() + 5);
      //    }
    }
  
    // Print out current samples
    print_currents();
  
    // Reset sensor measurements
    for (int k = 0; k < 4; k++) {
      sensor_vals[k] = 0;
      prev_error[k] = error[k];
    }

    sample = 0;
  }
}

static float servo0_deg;
static float servo1_deg;
static float servo2_deg;
static float servo3_deg;

//COROUTINE(control_servo0) {
//  COROUTINE_LOOP() {
//    move_servo0(servo0_deg);
//    COROUTINE_DELAY(15);
//  }
//}
//
//COROUTINE(control_servo1) {
//  COROUTINE_LOOP() {
//    move_servo1(servo1_deg);
//    COROUTINE_DELAY(15);
//  }
//}
//
//COROUTINE(control_servo2) {
//  COROUTINE_LOOP() {
//    move_servo2(servo2_deg);
//    COROUTINE_DELAY(15);
//  }
//}

//COROUTINE(control_servo3) {
//  COROUTINE_LOOP() {
//    move_servo3(servo3_deg);
//    COROUTINE_DELAY(15);
//  }
//}

COROUTINE(straightline) {
  static float deg = 0.0;
  static bool increasing = true;
  COROUTINE_LOOP() {
    move_to(0, 100, 0, 1, 90);
    COROUTINE_DELAY(5000);
    move_to(0, 192, 0, 1, 90);
    COROUTINE_DELAY(5000);
//    if (increasing) {
//      for (deg = 16.00; deg <= 57.15; deg += 0.15) {
//        float deg2 = -deg + 90 + rad2deg(acos((LEN_2 - LEN_0 * cos(deg2rad(deg)))/LEN_1));
//        move_servo0(deg);
////        servo0_deg = deg;
//        move_servo1(deg2);
////        servo1_deg = deg2;
//        move_servo2(360.0 - (deg + deg2));
////        servo2_deg = 360.0 - (deg + deg2);
//        if (deg + 0.15 > 57.15) {
//          increasing = false;
//        }
//        COROUTINE_DELAY(15);
//      }
//    } else {
//      for (deg = 57.15; deg >= 16.00; deg -= 0.15) {
//        float deg2 = -deg + 90 + rad2deg(acos((LEN_2 - LEN_0 * cos(deg2rad(deg)))/LEN_1));
//        move_servo0(deg);
////        servo0_deg = deg;
//        move_servo1(deg2);
////        servo1_deg = deg2;
//        move_servo2(360.0 - (deg + deg2));
////        servo2_deg = 360.0 - (deg + deg2);
//        if (deg - 0.15 < 16.00) {
//          increasing = true;
//        }
//        COROUTINE_DELAY(15);  
//      }
//    }
  }
}

void loop() {
  CoroutineScheduler::loop();
}
