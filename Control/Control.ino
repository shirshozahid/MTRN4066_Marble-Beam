// ==== Old Code ====
// volatile long pulseCount = 0;
// unsigned long prevTime = 0;
// unsigned long prevPulseCount = 0;
// float rpm = 0;

// unsigned long startTime = 0;  // Variable to store the time when the motor should start
// const unsigned long motorStartDelay = 5000; // 3 seconds delay before starting the motor
// const int UPDATE_INTERVAL = 50;




// ==== Pin Definitions ====
#define ENA 10
#define IN1 9
#define IN2 8
#define ENCODERA_PIN 19
#define ENCODERB_PIN 18
#define POT_PIN 17  // Potentiometer analog pin

// ==== Encoder Variables ====
volatile long pulseCount = 0;
const int PULSES_PER_REV = 134*4;  // Adjust to your encoder
float beamAngleRad = 0;

// ==== Control Gains ====
const float Kp = 30*0;
const float Kd = 6000*0;
const float Ktheta = 35;

// ==== Control Variables ====
//float x_marble = 0;
float v_marble = 0;
float theta = 0;
float x_last = 0;
const float x_want = 0.0;

// ==== Timing ====
const float dt = 10; //10 ms
unsigned long prevMillis = 0;

// ==== Encoder Interrupts ====
 void encoderISRA() {
    if(digitalRead(ENCODERA_PIN) == HIGH) { //Rising edge in A
      if (digitalRead(ENCODERB_PIN) == HIGH) { 
      pulseCount--;
    } else {
      pulseCount++;
    }
  } else { //Falling edge in B
    if (digitalRead(ENCODERB_PIN) == LOW) { 
        pulseCount--;
    } else {
        pulseCount++;
    }
  }
}


void encoderISRB() {
    if(digitalRead(ENCODERB_PIN) == HIGH) { //Rising edge in B
      if (digitalRead(ENCODERA_PIN) == HIGH) { 
      pulseCount++;
    } else {
      pulseCount--;
    }
  } else { //Falling edge in A
    if (digitalRead(ENCODERA_PIN) == LOW) { 
        pulseCount++;
    } else {
        pulseCount--;
    }
  }
}

// ==== Setup ====
void setup() {
    Serial.begin(115200);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENCODERA_PIN, INPUT);
    pinMode(ENCODERB_PIN, INPUT);
    //pinMode(POT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODERA_PIN), encoderISRA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERB_PIN), encoderISRB, CHANGE);
}

//==== Helper Functions ====
float readMarblePosition() {
  int potVal = map(analogRead(POT_PIN), 0, 1023, -850, 850);
  return potVal;
}

float readBeamAngle() {
  float degPerPulse = 360 / PULSES_PER_REV;
  float angleDeg = pulseCount * degPerPulse;
  return angleDeg * PI / 180.0; // radians
}

void updateVelocity(float x) {
  v_marble = (x - x_last) / 10;
  x_last = x;
}

float computeControl(float x, float v, float theta) {
  float V = Kp * (x - x_want) + (Kd * v) - (Ktheta * theta);
  V = constrain(V, -12.0 , 12.0);
  return V;
}

void applyVoltage(float V) {
  int pwm = (int)(abs(V) / 12.0 * 255.0);
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENA, pwm);
  if (V >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}


void loop() {
  unsigned long currentTime = millis();
  if(currentTime - prevMillis >= 10) {
    prevMillis = currentTime;

    float x_marble = readMarblePosition() / 10000;
    Serial.print(x_marble);
    theta = readBeamAngle();
    updateVelocity(x_marble);
    float V_control = computeControl(x_marble, v_marble, theta);
    applyVoltage(V_control);
    Serial.print("|");
    Serial.println(theta);

    // ==== Debug Print ====
    // Serial.print("x: "); Serial.print(x_marble, 4);
    // Serial.print(" | v: "); Serial.print(v_marble, 4);
    // Serial.print(" | theta: "); Serial.print(theta, 4);
    // Serial.print(" | V: "); Serial.print(V_control, 2);
  }
}
    // // Start motor after 3 seconds
    // if (currentTime - startTime >= motorStartDelay && (analogRead(ENA) == 0)) {
    //     analogWrite(ENA, 255);  // Turn on motor at full speed (255)
    //     digitalWrite(IN1, HIGH);
    //     digitalWrite(IN2, LOW);
    // }

    // // Calculate RPM every second (1000 ms)
    // if (currentTime - prevTime >= UPDATE_INTERVAL) {
    //     unsigned long elapsedPulses = (pulseCount - prevPulseCount);
    //     prevPulseCount = pulseCount;
    //     prevTime = currentTime;

    //     // Calculate RPM (Assuming pulse per revolution, adjust if needed)
    //     rpm = ((float)elapsedPulses / PULSES_PER_REV) * (60 * (1000 / UPDATE_INTERVAL)); 
        
    //     Serial.println(pulseCount);

