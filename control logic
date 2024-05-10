#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
#define trigPin1 2
#define echoPin1 4 
long duration, distance, Sensor1;

// Define the pins for motor control
#define MOTOR1_PIN1 5
#define MOTOR1_PIN2 6
#define MOTOR2_PIN1 3
#define MOTOR2_PIN2 11

// Define the pins for the LDR sensors
#define LEFT_LDR_PIN A0
#define RIGHT_LDR_PIN A1

// Define the threshold values for left and right light detection
#define LEFT_LIGHT_THRESHOLD 600
#define RIGHT_LIGHT_THRESHOLD 600
#define PWM_DATA 150
void moveForward() {
  analogWrite(MOTOR1_PIN1, PWM_DATA);
  analogWrite(MOTOR1_PIN2, 0);
  analogWrite(MOTOR2_PIN1, PWM_DATA);
  analogWrite(MOTOR2_PIN2, 0);
}

// Function to turn left
void turnLeft() {
  analogWrite(MOTOR1_PIN1, 0);
  analogWrite(MOTOR1_PIN2, PWM_DATA);
  analogWrite(MOTOR2_PIN1, PWM_DATA);
  analogWrite(MOTOR2_PIN2, 0);
}

// Function to turn right
void turnRight() {
  analogWrite(MOTOR1_PIN1, PWM_DATA);
  analogWrite(MOTOR1_PIN2, 0);
  analogWrite(MOTOR2_PIN1, 0);
  analogWrite(MOTOR2_PIN2, PWM_DATA);
}

// Function to stop the robot
void stopMoving() {
  analogWrite(MOTOR1_PIN1, 0);
  analogWrite(MOTOR1_PIN2, 0);
  analogWrite(MOTOR2_PIN1, 0);
  analogWrite(MOTOR2_PIN2, 0);
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(trigPin1, OUTPUT);
pinMode(echoPin1, INPUT);
}

void loop() {
  for (pos = 40; pos <=100 ; pos += 1) { // goes from 40 degrees to 100 degree1
    light_Following();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  
  for (pos = 100; pos >= 40; pos -= 1) { // goes from 100 degrees to 40 degrees
    light_Following();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  
}
void light_Following()
{
  SonarSensor(trigPin1, echoPin1);
    int Sensor1= distance;
   int leftLdrValue = analogRead(LEFT_LDR_PIN);
    int rightLdrValue = analogRead(RIGHT_LDR_PIN);

  // Print the LDR sensor values (for debugging)
  Serial.print("Left LDR value: ");
  Serial.print(leftLdrValue);
  Serial.print("Right LDR value: ");
  Serial.print(rightLdrValue);
  Serial.print("Distance: ");
  Serial.println(Sensor1);
  if(Sensor1<50)
  {
     turnRight();
    Serial.println("obstrucle");
    delay(1000);
    stopMoving();
    Serial.println("stop");
    delay(1000);
  }

  // Check which direction has brighter light
  if (leftLdrValue < LEFT_LIGHT_THRESHOLD && rightLdrValue < RIGHT_LIGHT_THRESHOLD) {
    // Both sides have bright light, move forward
    moveForward();
    Serial.println("Forward");
  } 
  else if (leftLdrValue < LEFT_LIGHT_THRESHOLD) {
    // Left side has brighter light, turn right
    turnRight();
    Serial.println("Right");
  } else if (rightLdrValue < RIGHT_LIGHT_THRESHOLD) {
    // Right side has brighter light, turn left
    turnLeft();
    Serial.println("left");
  } 
else {
    // No bright light detected, stop
    stopMoving();
    Serial.println("stop");
  }
}
void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;
}
