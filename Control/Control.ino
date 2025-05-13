// ==== Pin Definitions ====
#define ENA 10
#define IN1 9
#define IN2 8
#define ENCODERA_PIN 19
#define ENCODERB_PIN 18
#define POT_PIN 17  // Potentiometer analog pin

// ==== Encoder Variables ====
volatile long pulseCount = 0;
const int PULSES_PER_REV = 134.4*4;  // Adjust to your encoder
float beamAngleRad = 0;

// ==== Control Gains ====
const float Kp = 100.0;
const float Kd = 14.0;
const float Ktheta = 20.0;

// ==== Control Variables ====
float x_marble = 0;
float v_marble = 0;
float theta = 0;
//float x_last = 0;
const float x_want = 0.0;

volatile long pulseCount = 0;
unsigned long prevTime = 0;
unsigned long prevPulseCount = 0;
float rpm = 0;

unsigned long startTime = 0;  // Variable to store the time when the motor should start
const unsigned long motorStartDelay = 5000; // 3 seconds delay before starting the motor
const int UPDATE_INTERVAL = 50;

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

void setup() {
    Serial.begin(115200);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENCODERA_PIN, INPUT);
    pinMode(ENCODERB_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODERA_PIN), encoderISRA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERB_PIN), encoderISRB, CHANGE);

    // Record the time when the system is powered on
    startTime = 0;
}

void loop() {
    unsigned long currentTime = millis();

    // Start motor after 3 seconds
    if (currentTime - startTime >= motorStartDelay && (analogRead(ENA) == 0)) {
        analogWrite(ENA, 255);  // Turn on motor at full speed (255)
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }

    // Calculate RPM every second (1000 ms)
    if (currentTime - prevTime >= UPDATE_INTERVAL) {
        unsigned long elapsedPulses = (pulseCount - prevPulseCount);
        prevPulseCount = pulseCount;
        prevTime = currentTime;

        // Calculate RPM (Assuming pulse per revolution, adjust if needed)
        rpm = ((float)elapsedPulses / PULSES_PER_REV) * (60 * (1000 / UPDATE_INTERVAL)); 
        
        Serial.println(pulseCount);
    }
}

