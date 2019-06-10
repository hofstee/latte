#include <Math.h>
#include <Servo.h>
#include <AceRoutine.h>

using namespace ace_routine;

Servo tilt;
Servo pan;
Servo rot1;
Servo rot2;

Servo servos[4];

#define DEBUG false
#define CURRENT false

//const int MIN_PWM = 553;
//const int MAX_PWM = 2425;
//const int MIN_PWM = 600;
//const int MAX_PWM = 2400;

int sensors[] = { A0, A1, A2, A3 };
unsigned int sensor_vals[4];
float current[4];
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
const float LEN_2 = 101.97; // pivot 2 to middle of 3-bracket
const float LEN_3 = 45.0;
const float LEN_4 = 15.0; // z-offset from pivot to pitcher tip

const float L00 = LEN_0 * LEN_0;
const float L11 = LEN_1 * LEN_1;
const float L01 = LEN_0 * LEN_1;

float wrap_range(float x, float min, float max) {
  return min + fmod(max - min + fmod(x - min, max - min), max - min);
}

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
    total_current += current[k];
    Serial.print(current[k]);
    Serial.print(",");
  }
  Serial.print(total_current);
  Serial.print("\n");
}

const float D0_MIN = 0.00;
const float D0_MAX = 140.00;
void move_servo0(float deg) {
  // deg in [0.00, 140.00]
  // 40 tooth driving a 48 tooth gear -> 1.2 gear ratio
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
  //  0.00 deg ->  652 us
  // 90.00 deg -> 1740 us
  #if DEBUG
    Serial.print("Moving servo 0 to ");
    Serial.println(deg);
  #endif
  servos[0].writeMicroseconds(round(652 + (deg * 1.2)/0.0992647));
}

const float D1_MIN = 60.00;
const float D1_MAX = 200.00;
void move_servo1(float deg) {
  // deg in [60.00, 200.00]
  // 40 tooth driving a 48 tooth gear -> 1.2 gear ratio
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
  //  90.00 deg -> 1000 us
  // 180.00 deg -> 2114 us
  #if DEBUG
    Serial.print("Moving servo 1 to ");
    Serial.println(deg);
  #endif
  servos[1].writeMicroseconds(round(-114 + (deg * 1.2)/0.0969469));
}

const float D2_MIN = 0.00;
const float D2_MAX = 180.00;
void move_servo2(float deg) {
  // deg in [0.00, 180.00]
  // Direct connection
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
  //   0.00 deg -> 2390 us
  //  90.00 deg -> 1460 us
  #if DEBUG
    Serial.print("Moving servo 2 to ");
    Serial.println(deg);
  #endif
  servos[2].writeMicroseconds(round(530 + (180.0-deg)/0.09677419));
}

void move_servo3(float deg) {
  // Direct connection
  // Servo characteristics: (in theory 0.102 deg/us, but not in practice)
//  servos[3].writeMicroseconds(1472); // roughly 90 deg
  servos[3].writeMicroseconds(round(590 + deg/0.102));
}

bool viable_pos(float d0, float d1, float d2) {
  #if DEBUG
    Serial.println(d0);
    Serial.println(d1);
    Serial.println(d2);
  #endif
  return D0_MIN <= d0 && d0 <= D0_MAX &&
         D1_MIN <= d1 && d1 <= D1_MAX &&
         D2_MIN <= d2 && d2 <= D2_MAX;
}

// TODO: easing and make servos move to target position over time
void move_to(float x, float y, float pan, float tilt) {
  // x-extent:
  // y-extent: 100mm - 192mm
  float pan_rad = deg2rad(pan);
  float s_pan = sin(pan_rad);
  float c_pan = cos(pan_rad);
  float tilt_rad = deg2rad(tilt);
  float s_tilt = sin(tilt_rad);
  float c_tilt = cos(tilt_rad);
  
  float offset_x = -LEN_2 * s_pan + LEN_3 * c_pan * s_tilt + LEN_4 * c_pan * c_tilt; // x-offset due to pitcher
  float offset_y = LEN_2 * c_pan + LEN_3 * s_pan * s_tilt + LEN_4 * s_pan * c_tilt; // y-offset due to pitcher

  float target_x = x - offset_x;
  float target_y = y - offset_y;

  #if DEBUG
    Serial.println(target_x);
    Serial.println(target_y);
  #endif

  bool lhs = true;

  float dist2 = target_x*target_x + target_y*target_y;
  float R = sqrt(dist2);
  float gamma = atan2(target_y, target_x);
  float beta = acos(wrap_range((dist2 - L00 - L11)/(-2*L01), -1.0, 1.0));
  float psi = M_PI - beta;
  float alpha = asin((LEN_1 * sin(psi)) / R);

  #if DEBUG
    Serial.print("R: ");
    Serial.println(R);
    Serial.print("gamma: ");
    Serial.println(gamma);
    Serial.print("stuff: ");
    Serial.println((dist2 - L00 - L11)/(-2*L01));
    Serial.print("beta: ");
    Serial.println(beta);
    Serial.print("psi: ");
    Serial.println(psi);
    Serial.print("alpha: ");
    Serial.println(alpha);
  #endif

  float lhs_deg0 = wrap_range(rad2deg(gamma + alpha), 0.0, 360.0);
  float lhs_deg1 = wrap_range(90.0 + rad2deg(-psi), 0.0, 360.0);
  float lhs_deg2 = wrap_range(270.0 - (lhs_deg0 + lhs_deg1) + pan, 0.0, 360.0);
  bool lhs_viable = viable_pos(lhs_deg0, lhs_deg1, lhs_deg2);
  
  float rhs_deg0 = wrap_range(rad2deg(gamma - alpha), 0.0, 360.0);
  float rhs_deg1 = wrap_range(90.0 + rad2deg(psi), 0.0, 360.0);
  float rhs_deg2 = wrap_range(270.0 - (rhs_deg0 + rhs_deg1) + pan, 0.0, 360.0);
  bool rhs_viable = viable_pos(rhs_deg0, rhs_deg1, rhs_deg2);

//  Serial.println(lhs_deg0);
//  Serial.println(lhs_deg1);
//  Serial.println(lhs_deg2);
//  Serial.println("===");
//  Serial.println(rhs_deg0);
//  Serial.println(rhs_deg1);
//  Serial.println(rhs_deg2);

  if (lhs_viable && rhs_viable) {
    // Pick the one where deg2 is closer to 90 degrees to give max clearance to pitcher
    if (abs(90.0 - rhs_deg2) <= abs(90.0 - lhs_deg2)) {
      move_servo0(rhs_deg0);
      move_servo1(rhs_deg1);
      move_servo2(rhs_deg2);      
    } else {
      move_servo0(lhs_deg0);
      move_servo1(lhs_deg1);
      move_servo2(lhs_deg2);      
    }
    move_servo3(tilt);
  } else if (rhs_viable) {
    move_servo0(rhs_deg0);
    move_servo1(rhs_deg1);
    move_servo2(rhs_deg2);
    move_servo3(tilt);
  } else if (lhs_viable) {
    move_servo0(lhs_deg0);
    move_servo1(lhs_deg1);
    move_servo2(lhs_deg2);
    move_servo3(tilt);
  } else {
    // Don't move.
    #if DEBUG
      Serial.print("Can't move to requested position: (");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.print(pan);
      Serial.print(", ");
      Serial.print(tilt);
      Serial.print(")\n");
    #else
      Serial.println("Can't move!");
    #endif
  }
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

float lerp(float t, float x0, float x1) {
  return x0 + t * (x1 - x0);
}

void heart() {
  // Use about 4oz of liquid
  
  // About 7 rotations over 5 seconds
  const unsigned long NUM_ROTATIONS = 7;
  const unsigned long FILL_DUR = 5000;
  const unsigned long FILL_STOP_TILT = 35;
  // About 5 seconds for the heart
  const unsigned long HEART_DUR = 5000;
  // About 1 second for the pull through
  const unsigned long PULL_DUR = 1000;

  unsigned long start_ms, cur_ms;

  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + 1000) {
    float tilt = map(cur_ms, start_ms, start_ms + 1000, 90, 40);
    float y = 140 + map(cur_ms, start_ms, start_ms + 1000, 0, 20);
    move_to(0, y, 90, tilt);
  }
  
  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + FILL_DUR) {
//    float t = map(cur_ms, start_ms, start_ms + 5000, 0.0, NUM_ROTATIONS*2*M_PI);
//    float x = 15 * cos(t);
//    float y = 150 + 10 * sin(t);

    int rot_ms = (cur_ms - start_ms) % (FILL_DUR/NUM_ROTATIONS);
    int quadrant = 4 * rot_ms / (FILL_DUR / NUM_ROTATIONS);
    float t = ((float)((4 * rot_ms) % (FILL_DUR / NUM_ROTATIONS))) / (FILL_DUR/NUM_ROTATIONS);
    float x, y;
    switch (quadrant) {
      case 0:
        x = lerp(t, 15, 0);
//        y = 150 + lerp(t, 0, 10);
        y = 150;
        break;
      case 1:
        x = lerp(t, 0, -15);
//        y = 150 + lerp(t, 10, 0);
        y = 150;
        break;
      case 2:
        x = lerp(t, -15, 0);
//        y = 150 + lerp(t, 0, -10);
        y = 150;
        break;
      case 3:
        x = lerp(t, 0, 15);
//        y = 150 + lerp(t, -10, 0);
        y = 150;
        break;
    }
    
    float tilt = map(cur_ms, start_ms, start_ms + 5000, 40, FILL_STOP_TILT);
    move_to(x, y, 90, tilt);
  }

  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + 500) {
    float tilt = map(cur_ms, start_ms, start_ms + 500, FILL_STOP_TILT, 45);
    move_to(0, 140, 90, tilt);
  }

  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + HEART_DUR) {
    float tilt = map(cur_ms, start_ms , start_ms + HEART_DUR, FILL_STOP_TILT, 10);
    move_to(0, 140, 90, tilt);
  }

  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + PULL_DUR) {
    float y = 140 + map(cur_ms, start_ms, start_ms + PULL_DUR, 0, 40);
    float tilt = map(cur_ms, start_ms , start_ms + PULL_DUR, 10, 0);
    move_to(0, y, 90, tilt);
  }
  

  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + 1000) {
    float tilt = map(cur_ms, start_ms, start_ms + 1000, 0, 20);
    move_to(0, 180, 90, tilt);
  }

  start_ms = millis();
  while ((cur_ms = millis()) < start_ms + 2000) {
    float tilt = map(cur_ms, start_ms, start_ms + 2000, 20, 90);
    float y = 140 + map(cur_ms, start_ms, start_ms + 2000, 40, 0);
    move_to(0, y, 90, tilt);
  }
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

//  move_servo1(100);
//  move_servo2(180);

//  move_to(0, 100 + LEN_3, 0, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 15, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 30, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 45, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 60, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 75, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 90, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 105, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 120, 90);
//  delay(500);
//  move_to(0, 100 + LEN_3, 135, 90);
//  delay(500);

//  move_to(0, 332.1, 90, 90);
//  move_to(0, 186.08, 0, 90);
//  move_to(-153.8, 268.98, 90, 90); // 90, 180, 90, 90
//  move_to(59.01, 223.83, 90, 90); // 0, 90, 90, 90
//  move_to(0, 332.1, 80, 90);

//  move_to(0, 206.7, 90, 90); // y-min 90 tilt for config 2
//  move_to(0, 125, 90, 90); // y-min 90 tilt for config 3
//  move_to(0, 125, 90, 0);
//  move_to(0, 190, 90, 0); // y-max 0 tilt

  CoroutineScheduler::setup();

  heart();
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
    
    // Compute currents
    for (int k = 0; k < 4; k++) {
      current[k] = compute_current(sensor_vals[k] / num_samples);
    }
  
    // Print out current samples
    #if CURRENT
      print_currents();
    #endif
  
    // Reset sensor measurements
    for (int k = 0; k < 4; k++) {
      sensor_vals[k] = 0;
      prev_error[k] = error[k];
    }
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

//COROUTINE(straightline) {
//  COROUTINE_LOOP() {
//    move_to(0, 100, 90, 90);
//    COROUTINE_DELAY(5000);
//    move_to(0, 192, 90, 90);
//    COROUTINE_DELAY(5000);
//  }
//}

COROUTINE(stationary) {
  COROUTINE_LOOP() {
    static float tilt = 45;
    for (float tilt = 45; tilt <= 90; tilt += 0.01) {
      move_to(0, 332.1, tilt, 90);
      COROUTINE_DELAY(1);
    }
    for (float tilt = 90; tilt >= 45; tilt -= 0.01) {
      move_to(0, 332.1, tilt, 90);
      COROUTINE_DELAY(1);
    }
  }
}

void loop() {
//  CoroutineScheduler::loop();
//  for (float tilt = 45; tilt <= 90; tilt += 0.01) {
//    move_to(0, 332.1, tilt, 90);
//  }
//  for (float tilt = 90; tilt >= 45; tilt -= 0.01) {
//    move_to(0, 332.1, tilt, 90);
//  }

//  #if DEBUG
//    for (float pan = 0; pan <= 90; pan += 1.0) {
//      move_to(0, 135, 90, pan);
//      delay(5);
//    }
//    for (float pan = 90; pan >= 0; pan -= 1.0) {
//      move_to(0, 135, 90, pan);
//      delay(5);
//    }
//  #else
//    for (float pan = 0; pan <= 90; pan += 0.1) {
//      move_to(0, 135, 90, pan);
//    }
//    for (float pan = 90; pan >= 0; pan -= 0.1) {
//      move_to(0, 135, 90, pan);
//    }
//  #endif

//  heart();
}
