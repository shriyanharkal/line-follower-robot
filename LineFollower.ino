// ================= SENSOR CONFIGURATION =================
const int NUM_SENSORS = 3;
const int sensorPins[NUM_SENSORS] = {A4, A1, A0}; // index 0 = left, 1 = middle, 2 = right
const int weights[NUM_SENSORS] = {-1, 0, 1};  // left, middle, right

const int LOST_THRESHOLD = 100; // below this total, we don't trust the line is there
const int NOISE_FLOOR = 15; // normalized units (0-1000) treated as "nothing"

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

int lastPosition = 0;
long lineStrength = 0;

unsigned long lastTime = 0;
int lastError = 0;

// ================= MOTOR CONFIGURATION =================
const int STBY = 4;
const int PWMA = 5, AIN1 = 7, AIN2 = 8; // left motor
const int PWMB = 6, BIN1 = 9, BIN2 = 10;  // right motor

const int MAX_SPEED = 150;
const float LEFT_TRIM = 0.78; // reduce left motor speed to compensate for the slower right motor

// ================= CONTROL CONFIGURATION =================
const float KP = 0.072; // proportional gain
const float KD = 0.1; // derivative gain
const int BASE_SPEED = 100;
const unsigned long LOOP_MS = 10; // running the controller every 10 ms
 

void calibrate() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;   
    sensorMax[i] = 0;   
  }
  unsigned long start = millis();
  while (millis() - start < 10000) {          // sweep for 10 seconds
    for (int i = 0; i < NUM_SENSORS; i++) {
      int r = analogRead(sensorPins[i]);
      if (r < sensorMin[i]) sensorMin[i] = r;
      if (r > sensorMax[i]) sensorMax[i] = r;
    }
  }
}


void setup() {
  Serial.begin(9600);
  Serial.println("Place the car on the line. Starting in 3 seconds...");
  delay(3000);
  Serial.println("Calibrating: slide the car across the line, left and right, for 10 seconds");
  calibrate();
  Serial.println("Done. Results:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("sensor "); Serial.print(i);
    Serial.print("  min "); Serial.print(sensorMin[i]);
    Serial.print("  max "); Serial.println(sensorMax[i]);
  }

  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  digitalWrite(STBY, HIGH);
  Serial.println("Calibrated. Put the car on the line. Starting in 3 seconds...");
  delay(3000);
}


// Returns sensor i as 0 (its own white) up to 1000 (its own black)
int readNormalized(int i) {
  int range = sensorMax[i] - sensorMin[i];
  if (range <= 0) return 0;                       // guard: the sensor saw no difference
  long v = (long)(analogRead(sensorPins[i]) - sensorMin[i]) * 1000 / range;
  return constrain(v, 0, 1000);
}


int readPosition() {
  int s[NUM_SENSORS];
  lineStrength = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = readNormalized(i);
    s[i] = (v > NOISE_FLOOR) ? v - NOISE_FLOOR : 0;
    lineStrength += s[i];
  }

  if (lineStrength < LOST_THRESHOLD) return lastPosition;   // line not seen: keep the last position

  long weighted = 0;
  for (int i = 0; i < NUM_SENSORS; i++) weighted += (long)s[i] * weights[i];
  lastPosition = weighted * 1000 / lineStrength;            // -1000 (left) to +1000 (right)
  return lastPosition;
}


void setMotorA(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speed = -speed; }
  analogWrite(PWMA, speed);
}

void setMotorB(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); speed = -speed; }
  analogWrite(PWMB, speed);
}

void drive(int left, int right) {
  setMotorA((int)(left * LEFT_TRIM));    // left motor (A)
  setMotorB(right);   // right motor (B)
}


void loop() {
  unsigned long now = millis();
  if (now - lastTime < LOOP_MS) return;     // not time yet: do nothing
  lastTime = now;

  int error = readPosition();
  int derivative = error - lastError;       // change in the last 10 ms
  lastError = error;

  int correction = KP * error + KD * derivative;
  drive(BASE_SPEED + correction, BASE_SPEED - correction);
}