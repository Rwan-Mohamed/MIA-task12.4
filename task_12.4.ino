// Pin Definitions
const int encoderPinA = 2;   // Encoder pin A
const int encoderPinB = 3;   // Encoder pin B
const int motorPinPWM = 9;   // PWM output to motor driver
const int motorPinDir = 8;   // Direction pin

// PID Control Parameters
double setpoint = 200;  // Desired speed (ticks per second)
double input = 0;       // Measured speed
double output = 0;      // Control output (PWM)
double error = 0, previous_error = 0;
double integral = 0;
double derivative = 0;
double Kp = 2, Ki = 5, Kd = 1;  // Tune these values

// Exponential Smoothing Filter Parameter
double filtered_output = 0;
double alpha = 0.2;  // Smoothing factor (0 < alpha <= 1)

// Encoder Variables
volatile long encoderTicks = 0;
long prevEncoderTicks = 0;
unsigned long prevTime = 0;

void setup() {
  // Initialize encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  
  // Initialize motor control pins
  pinMode(motorPinPWM, OUTPUT);
  pinMode(motorPinDir, OUTPUT);
  
  // Start Serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Calculate speed (ticks per second)
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - prevTime;
  if (elapsedTime >= 100) {  // Update every 100 ms
    long currentEncoderTicks = encoderTicks;
    long deltaTicks = currentEncoderTicks - prevEncoderTicks;
    input = (double)deltaTicks / (elapsedTime / 1000.0);  // Speed in ticks per second
    
    // Store current time and encoder value
    prevTime = currentTime;
    prevEncoderTicks = currentEncoderTicks;
    
    // Compute PID
    PID_compute();
    
    // Apply Exponential Smoothing Filter (Soft Start)
    filtered_output = exponentialSmoothing(output, filtered_output);
    
    // Apply the filtered output to control motor speed
    setMotorSpeed(filtered_output);
    
    // Debugging info
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" Input: ");
    Serial.print(input);
    Serial.print(" Output: ");
    Serial.print(output);
    Serial.print(" Filtered Output: ");
    Serial.println(filtered_output);
  }
}

// Encoder ISR (Interrupt Service Routine)
void encoderISR() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);
  if (stateA == stateB) {
    encoderTicks++;  // Clockwise
  } else {
    encoderTicks--;  // Counterclockwise
  }
}

// Manual PID compute function
void PID_compute() {
  // Calculate error
  error = setpoint - input;
  
  // Proportional term
  double P = Kp * error;
  
  // Integral term
  integral += error;
  double I = Ki * integral;
  
  // Derivative term
  derivative = error - previous_error;
  double D = Kd * derivative;
  
  // Combine all terms
  output = P + I + D;
  
  // Limit output to valid PWM range (0-255)
  if (output > 255) output = 255;
  if (output < 0) output = 0;
  
  // Save the current error for the next loop (for D term)
  previous_error = error;
}

// Exponential Smoothing Filter function
double exponentialSmoothing(double current_output, double previous_output) {
  return alpha * current_output + (1 - alpha) * previous_output;
}

// Function to set motor speed and direction
void setMotorSpeed(double speed) {
  // Set PWM output
  analogWrite(motorPinPWM, speed);
  
  // Set direction based on speed (can be adjusted based on specific motor setup)
  if (speed > 0) {
    digitalWrite(motorPinDir, HIGH);  // Forward
  } else {
    digitalWrite(motorPinDir, LOW);   // Reverse
  }
}
