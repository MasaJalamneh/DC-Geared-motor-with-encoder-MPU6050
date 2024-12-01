// motor driver pins
int motorPin1 = 4;  // motor driver IN1 pin connected to D4
int motorPin2 = 5;  // motor driver IN2 pin connected to D5
int enablePin = 9;  // motor driver Enable pin connected to D9 

// encoder pins
int encoderPinA = 2;  // encoder A 'c1' connected to pin 2
int encoderPinB = 3;  // encoder B 'c2' connected to pin 3

// joystick pins
int joyX = A0;  // joystick X-axis connected to A0
int joyY = A1;  // joystick Y-axis connected to A1 (optional, only worked on X-axis)
int joystickButton = 8;  // joystick switch (SW) connected to pin 8 (optional)

volatile int encoderPos = 0;
int lastEncoded = 0;
bool motorRunning = true;  // to track motor state (running or stopped)
int lastEncoderPos = 0;    // to track the last encoder position to determine direction
String motorDirection = "Stopped";  // to track motor direction

// Pulses per Revolution 'PPR'
#define ENCODER_PPR 360  // 360 pulses per revolution

// pin's setup
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);  // set ENABLE pin --> output

  // set encoder pins 'A, B' --> input
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // set joystick button pin --> input
  pinMode(joystickButton, INPUT_PULLUP);  // Use internal pull-up resistor

  // interrupts for encoder reading
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  // start serial communication
  Serial.begin(9600); 

  // enable motor driver (enable pin => HIGH)
  digitalWrite(enablePin, HIGH);
}

void loop() {
  // read joystick X and Y values
  int joystickX = analogRead(joyX);  //  X-axis value (for speed and direction)
  int joystickY = analogRead(joyY);  //  Y-axis value (optional, didn't use it)

  // check if the joystick button is pressed (SW pin)
  if (digitalRead(joystickButton) == LOW) {  // button pressed (pressed => LOW)
    motorRunning = !motorRunning;  // toggle motor state (start/stop)
    delay(300);  // to avoid multiple toggles
  }

  // ff motor is running --> control the motor
  if (motorRunning) {
    // calculate motor speed: absolute value of joystick X minus 512
    int motorSpeed = abs(joystickX - 512);  // get the absolute joystick value and subtract 512

    // control motor direction and speed based on joystick X-axis value
    if (joystickX > 512) {  // move forward (clockwise)
      digitalWrite(motorPin1, HIGH);  // IN1 HIGH
      digitalWrite(motorPin2, LOW);   // IN2 LOW
      analogWrite(enablePin, motorSpeed);  // adjust speed using PWM on enable pin
      motorDirection = "Forward";  // update motor direction for serial print
    } else if (joystickX < 512) {  // move backward (counterclockwise)
      digitalWrite(motorPin1, LOW);   // IN1 LOW
      digitalWrite(motorPin2, HIGH);  // IN2 HIGH
      analogWrite(enablePin, motorSpeed);  // adjust speed using PWM on enable pin
      motorDirection = "Backward";  // update motor direction for serial print
    } else {  // stop motor if joystick is centered (joystickX == 512)
      digitalWrite(motorPin1, LOW);   // IN1 LOW
      digitalWrite(motorPin2, LOW);   // IN2 LOW
      analogWrite(enablePin, 0);  // stop motor by setting enable pin PWM to 0
      motorDirection = "Stopped";  // update motor direction for serial print
    }
  } else {
    // if motor is stopped, ensure it's fully stopped 
    digitalWrite(motorPin1, LOW);  // IN1 LOW
    digitalWrite(motorPin2, LOW);  // IN2 LOW
    analogWrite(enablePin, 0);     // stop motor by disabling the enable pin
    motorDirection = "Stopped";  // update motor direction for serial print
  }
  // serial print
  // print encoder position, angle, direction, and joystick data to the serial monitor
  // position
  Serial.print("Encoder Position: ");
  Serial.print(encoderPos);

  // calculate the angle of rotation (in degrees) based on encoder position
  // angle
  float angle = (float(encoderPos) / ENCODER_PPR) * 360.0;
  Serial.print(" | Angle: ");
  Serial.print(angle, 2); 

  // direction
  Serial.print(" | Direction: ");
  Serial.println(motorDirection);

  // joystick
  Serial.print("Joystick X: ");
  Serial.print(joystickX);
  Serial.print(" | Joystick Y: ");
  Serial.println(joystickY);

  delay(500);  // to make the serial output readable and prevent flooding
}

// function to update encoder position and direction
void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // Most-Significant-Bit
  int LSB = digitalRead(encoderPinB); // Least-Significant-Bit

  int encoded = (MSB << 1) | LSB;  // combine the bits
  int sum = (lastEncoded << 2) | encoded;  // combine the previous and current state

  // update encoder position based on changes
  if (sum == 0b01 || sum == 0b10) {
    encoderPos++;  // clockwise rotation
    motorDirection = "Forward";  // update motor direction
  }
  if (sum == 0b11 || sum == 0b00) {
    encoderPos--;  // counter-clockwise rotation
    motorDirection = "Backward";  // update motor direction
  }

  lastEncoded = encoded;  // then store the current encoder state
}