#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

const int motorLeftPWM = 5;   // PWM pin for left motor
const int motorRightPWM = 8;  // PWM pin for right motor
const int motorLeftDir = 4;   // Direction pin for left motor
const int motorRightDir = 7;  // Direction pin for right motor

float linearVelocity = 0;
float angularVelocity = 0;
float trackWidth = 0.4;  // Distance between the two wheels (meters)
float wheelRadius = 0.125;  // Radius of the wheels (meters)

// PWM limits
int maxPWM = 255;   // Max PWM value
int minPWM = -255;  // Min PWM value

// Function to set motor speeds
void setMotorSpeed(int motorPinPWM, int motorPinDir, float speed) {
  if (speed > 0) {
    digitalWrite(motorPinDir, HIGH);
  } else {
    digitalWrite(motorPinDir, LOW);
  }
  analogWrite(motorPinPWM, constrain(abs(speed), 0, maxPWM));
}

// Callback for receiving the velocity message
void velocityCallback(const std_msgs::Float32MultiArray& msg) {
  linearVelocity = msg.data[0];   // Linear velocity (m/s)
  angularVelocity = msg.data[1];  // Angular velocity (rad/s)

  // Compute the individual wheel speeds
  float v_left = linearVelocity - angularVelocity*(trackWidth/2.0);
  float v_right = linearVelocity + angularVelocity*(trackWidth/2.0);

  // Convert to wheel RPM (optional, for tuning purposes)
  float leftRPM = (v_left/(2*PI*wheelRadius))*60.0;
  float rightRPM = (v_right/(2*PI*wheelRadius))*60.0;

  // Convert velocities to PWM values
  int leftPWM = (int)(v_left*maxPWM);   // Scale to the PWM range
  int rightPWM = (int)(v_right*maxPWM);

  // Set motor speeds
  setMotorSpeed(motorLeftPWM, motorLeftDir, leftPWM);
  setMotorSpeed(motorRightPWM, motorRightDir, rightPWM);

  // For debugging
  Serial.print("Linear Velocity: ");
  Serial.println(linearVelocity);
  Serial.print("Angular Velocity: ");
  Serial.println(angularVelocity);
  Serial.print("Left Wheel PWM: ");
  Serial.println(leftPWM);
  Serial.print("Right Wheel PWM: ");
  Serial.println(rightPWM);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> sub("cmd_vel_array", velocityCallback);

void setup() {
  Serial.begin(9600);

  // Motor pins setup
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightDir, OUTPUT);

  // Initialize ROS node
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);  // Small delay to avoid overwhelming the communication
}
